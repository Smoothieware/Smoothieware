#pragma GCC diagnostic ignored "-Wstrict-aliasing"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wcast-align"

#include "CommandQueue.h"

#include "Kernel.h"
#include "Config.h"
#include "SlowTicker.h"

#include "Network.h"
#include "PublicDataRequest.h"
#include "PlayerPublicAccess.h"
#include "net_util.h"
#include "uip_arp.h"
#include "clock-arch.h"
#include "NetworkPublicAccess.h"
#include "checksumm.h"
#include "ConfigValue.h"

#include "uip.h"
#include "telnetd.h"
#include "webserver.h"
#include "dhcpc.h"
#include "sftpd.h"

#ifndef NOPLAN9
#include "plan9.h"
#endif

#include <mri.h>

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#define network_enable_checksum CHECKSUM("enable")
#define network_webserver_checksum CHECKSUM("webserver")
#define network_telnet_checksum CHECKSUM("telnet")
#define network_plan9_checksum CHECKSUM("plan9")
#define network_mac_override_checksum CHECKSUM("mac_override")
#define network_ip_address_checksum CHECKSUM("ip_address")
#define network_hostname_checksum CHECKSUM("hostname")
#define network_ip_gateway_checksum CHECKSUM("ip_gateway")
#define network_ip_mask_checksum CHECKSUM("ip_mask")

extern "C" void uip_log(char *m)
{
    printf("uIP log message: %s\n", m);
}

static Network* theNetwork;

Network::Network()
{
    theNetwork= this;
    ethernet = new LPC17XX_Ethernet();
    tickcnt= 0;
    sftpd= NULL;
    hostname = NULL;
    plan9_enabled= false;
    command_q= CommandQueue::getInstance();
}

Network::~Network()
{
    delete ethernet;
    if (hostname != NULL) {
        delete hostname;
    }
    theNetwork= nullptr;
}

static uint32_t getSerialNumberHash()
{
#define IAP_LOCATION 0x1FFF1FF1
    uint32_t command[1];
    uint32_t result[5];
    typedef void (*IAP)(uint32_t *, uint32_t *);
    IAP iap = (IAP) IAP_LOCATION;

    __disable_irq();

    command[0] = 58;
    iap(command, result);
    __enable_irq();
    return crc32((uint8_t *)&result[1], 4 * 4);
}

static bool parse_ip_str(const string &s, uint8_t *a, int len, int base=10, char sep = '.')
{
    int p = 0;
    const char *n;
    for (int i = 0; i < len; i++) {
        if (i < len - 1) {
            size_t o = s.find(sep, p);
            if (o == string::npos) return false;
            n = s.substr(p, o - p).c_str();
            p = o + 1;
        } else {
            n = s.substr(p).c_str();
        }
        a[i] = (int)strtol(n, NULL, base);
    }
    return true;
}

static bool parse_hostname(const string &s)
{
    const std::string::size_type str_len = s.size();
    if(str_len > 63){
        return false;
    }
    for (unsigned int i = 0; i < str_len; i++) {
        const char c = s.at(i);
        if(!(c >= 'a' && c <= 'z')
                && !(c >= 'A' && c <= 'Z')
                && !(i != 0 && c >= '0' && c <= '9')
                && !(i != 0 && i != str_len - 1 && c == '-')){
            return false;
        }
    }
    return true;
}

void Network::on_module_loaded()
{
    if ( !THEKERNEL->config->value( network_checksum, network_enable_checksum )->by_default(false)->as_bool() ) {
        // as not needed free up resource
        delete this;
        return;
    }

    webserver_enabled = THEKERNEL->config->value( network_checksum, network_webserver_checksum, network_enable_checksum )->by_default(false)->as_bool();
    telnet_enabled = THEKERNEL->config->value( network_checksum, network_telnet_checksum, network_enable_checksum )->by_default(false)->as_bool();
    plan9_enabled = THEKERNEL->config->value( network_checksum, network_plan9_checksum, network_enable_checksum )->by_default(false)->as_bool();
    string mac = THEKERNEL->config->value( network_checksum, network_mac_override_checksum )->by_default("")->as_string();
    if (mac.size() == 17 ) { // parse mac address
        if (!parse_ip_str(mac, mac_address, 6, 16, ':')) {
            printf("Invalid MAC address: %s\n", mac.c_str());
            printf("Network not started due to errors in config");
            return;
        }

    } else {   // autogenerate
        uint32_t h = getSerialNumberHash();
        mac_address[0] = 0x00;   // OUI
        mac_address[1] = 0x1F;   // OUI
        mac_address[2] = 0x11;   // OUI
        mac_address[3] = 0x02;   // Openmoko allocation for smoothie board
        mac_address[4] = 0x04;   // 04-14  03 bits -> chip id, 1 bits -> hashed serial
        mac_address[5] = h & 0xFF; // 00-FF  8bits -> hashed serial
    }

    ethernet->set_mac(mac_address);

    // get IP address, mask and gateway address here....
    string s = THEKERNEL->config->value( network_checksum, network_ip_address_checksum )->by_default("auto")->as_string();
    if (s == "auto") {
        use_dhcp = true;
        s = THEKERNEL->config->value( network_checksum, network_hostname_checksum )->as_string();
        if (!s.empty()) {
            if(parse_hostname(s)){
                hostname = new char [s.length() + 1];
                strcpy(hostname, s.c_str());
            }else{
                printf("Invalid hostname: %s\n", s.c_str());
            }
        }
    } else {
        bool bad = false;
        use_dhcp = false;
        if (!parse_ip_str(s, ipaddr, 4)) {
            printf("Invalid IP address: %s\n", s.c_str());
            bad = true;
        }
        s = THEKERNEL->config->value( network_checksum, network_ip_mask_checksum )->by_default("255.255.255.0")->as_string();
        if (!parse_ip_str(s, ipmask, 4)) {
            printf("Invalid IP Mask: %s\n", s.c_str());
            bad = true;
        }
        s = THEKERNEL->config->value( network_checksum, network_ip_gateway_checksum )->by_default("192.168.3.1")->as_string();
        if (!parse_ip_str(s, ipgw, 4)) {
            printf("Invalid IP gateway: %s\n", s.c_str());
            bad = true;
        }
        if (bad) {
            printf("Network not started due to errors in config");
            return;
        }
    }

    THEKERNEL->add_module( ethernet );
    THEKERNEL->slow_ticker->attach( 100, this, &Network::tick );

    // Register for events
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GET_PUBLIC_DATA);

    this->init();
}

void Network::on_get_public_data(void* argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(network_checksum)) return;

    if(pdr->second_element_is(get_ip_checksum)) {
        pdr->set_data_ptr(this->ipaddr);
        pdr->set_taken();

    }else if(pdr->second_element_is(get_ipconfig_checksum)) {
        // NOTE caller must free the returned string when done
        char buf[200];
        int n1= snprintf(buf,             sizeof(buf),         "IP Addr: %d.%d.%d.%d\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
        int n2= snprintf(&buf[n1],       sizeof(buf)-n1,       "IP GW: %d.%d.%d.%d\n", ipgw[0], ipgw[1], ipgw[2], ipgw[3]);
        int n3= snprintf(&buf[n1+n2],    sizeof(buf)-n1-n2,    "IP mask: %d.%d.%d.%d\n", ipmask[0], ipmask[1], ipmask[2], ipmask[3]);
        int n4= snprintf(&buf[n1+n2+n3], sizeof(buf)-n1-n2-n3, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
        char *str = (char *)malloc(n1+n2+n3+n4+1);
        memcpy(str, buf, n1+n2+n3+n4);
        str[n1+n2+n3+n4]= '\0';
        pdr->set_data_ptr(str);
        pdr->set_taken();
    }
}

uint32_t Network::tick(uint32_t dummy)
{
    do_tick();
    tickcnt++;
    return 0;
}

void Network::on_idle(void *argument)
{
    if (!ethernet->isUp()) return;

    int len= sizeof(uip_buf); // set maximum size
    if (ethernet->_receive_frame(uip_buf, &len)) {
        uip_len = len;
        this->handlePacket();

    } else {

        if (timer_expired(&periodic_timer)) { /* no packet but periodic_timer time out (0.1s)*/
            timer_reset(&periodic_timer);

            for (int i = 0; i < UIP_CONNS; i++) {
                uip_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    tapdev_send(uip_buf, uip_len);
                }
            }

#if UIP_CONF_UDP
            for (int i = 0; i < UIP_UDP_CONNS; i++) {
                uip_udp_periodic(i);
                /* If the above function invocation resulted in data that
                   should be sent out on the network, the global variable
                   uip_len is set to a value > 0. */
                if (uip_len > 0) {
                    uip_arp_out();
                    tapdev_send(uip_buf, uip_len);
                }
            }
#endif
        }
/*
        This didn't work actually made it worse,it should have worked though
        else{
            // TODO if the command queue is below a certain amount we should poll any stopped connections
            if(command_q->size() < 4) {
                for (struct uip_conn *connr = &uip_conns[0]; connr <= &uip_conns[UIP_CONNS - 1]; ++connr) {
                    if(uip_stopped(connr)){
                        // Force a poll of this
                        printf("Force poll of connection\n");
                        uip_poll_conn(connr);
                    }
                }
            }
        }
*/
        /* Call the ARP timer function every 10 seconds. */
        if (timer_expired(&arp_timer)) {
            timer_reset(&arp_timer);
            uip_arp_timer();
        }
    }
}

void Network::setup_servers()
{
    if (webserver_enabled) {
        // Initialize the HTTP server, listen to port 80.
        httpd_init();
        printf("Webserver initialized\n");
    }

    if (telnet_enabled) {
        // Initialize the telnet server
        Telnetd::init();
        printf("Telnetd initialized\n");
    }

#ifndef NOPLAN9
    if (plan9_enabled) {
        // Initialize the plan9 server
        Plan9::init();
        printf("Plan9 initialized\n");
    }
#endif

    // sftpd service, which is lazily created on reciept of first packet
    uip_listen(HTONS(115));
}

extern "C" void dhcpc_configured(const struct dhcpc_state *s)
{
    printf("Got IP address %d.%d.%d.%d\n",
           uip_ipaddr1(&s->ipaddr), uip_ipaddr2(&s->ipaddr),
           uip_ipaddr3(&s->ipaddr), uip_ipaddr4(&s->ipaddr));
    printf("Got netmask %d.%d.%d.%d\n",
           uip_ipaddr1(&s->netmask), uip_ipaddr2(&s->netmask),
           uip_ipaddr3(&s->netmask), uip_ipaddr4(&s->netmask));
    printf("Got DNS server %d.%d.%d.%d\n",
           uip_ipaddr1(&s->dnsaddr), uip_ipaddr2(&s->dnsaddr),
           uip_ipaddr3(&s->dnsaddr), uip_ipaddr4(&s->dnsaddr));
    printf("Got default router %d.%d.%d.%d\n",
           uip_ipaddr1(&s->default_router), uip_ipaddr2(&s->default_router),
           uip_ipaddr3(&s->default_router), uip_ipaddr4(&s->default_router));
    printf("Lease expires in %ld seconds\n", ntohl(s->lease_time));

    theNetwork->dhcpc_configured(s->ipaddr, s->netmask, s->default_router);
}

void Network::dhcpc_configured(uint32_t ipaddr, uint32_t ipmask, uint32_t ipgw)
{
    memcpy(this->ipaddr, &ipaddr, 4);
    memcpy(this->ipmask, &ipmask, 4);
    memcpy(this->ipgw, &ipgw, 4);

    uip_sethostaddr((u16_t*)this->ipaddr);
    uip_setnetmask((u16_t*)this->ipmask);
    uip_setdraddr((u16_t*)this->ipgw);

    setup_servers();
}

void Network::init(void)
{
    // two timers for tcp/ip
    timer_set(&periodic_timer, CLOCK_SECOND / 2); /* 0.5s */
    timer_set(&arp_timer, CLOCK_SECOND * 10);   /* 10s */

    // Initialize the uIP TCP/IP stack.
    uip_init();

    uip_setethaddr(mac_address);

    if (!use_dhcp) { // manual setup of ip
        uip_ipaddr_t tip;  /* local IP address */
        uip_ipaddr(tip, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
        uip_sethostaddr(tip);    /* host IP address */
        printf("IP Addr: %d.%d.%d.%d\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);

        uip_ipaddr(tip, ipgw[0], ipgw[1], ipgw[2], ipgw[3]);
        uip_setdraddr(tip);  /* router IP address */
        printf("IP GW: %d.%d.%d.%d\n", ipgw[0], ipgw[1], ipgw[2], ipgw[3]);

        uip_ipaddr(tip, ipmask[0], ipmask[1], ipmask[2], ipmask[3]);
        uip_setnetmask(tip); /* mask */
        printf("IP mask: %d.%d.%d.%d\n", ipmask[0], ipmask[1], ipmask[2], ipmask[3]);
        setup_servers();

    }else{
    #if UIP_CONF_UDP
        dhcpc_init(mac_address, sizeof(mac_address), hostname);
        dhcpc_request();
        printf("Getting IP address....\n");
    #endif
    }
}

void Network::on_main_loop(void *argument)
{
    // issue commands here if any available
    // while(command_q->pop()) {
    //     // keep feeding them until empty
    // }

    // issue one comamnd per iteration of main loop like USB serial does
    command_q->pop();

}

extern "C" const char *get_query_string()
{
    return THEKERNEL->get_query_string().c_str();
}

// select between webserver and telnetd server
extern "C" void app_select_appcall(void)
{
    switch (uip_conn->lport) {
        case HTONS(80):
            if (theNetwork->webserver_enabled) httpd_appcall();
            break;

        case HTONS(23):
            if (theNetwork->telnet_enabled) Telnetd::appcall();
            break;

#ifndef NOPLAN9
        case HTONS(564):
            if (theNetwork->plan9_enabled) Plan9::appcall();
            break;
#endif

        case HTONS(115):
            if(theNetwork->sftpd == NULL) {
                theNetwork->sftpd= new Sftpd();
                theNetwork->sftpd->init();
                printf("Created sftpd service\n");
            }
            theNetwork->sftpd->appcall();
            break;

        default:
            printf("unknown app for port: %d\n", uip_conn->lport);

    }
}

void Network::tapdev_send(void *pPacket, unsigned int size)
{
    memcpy(ethernet->request_packet_buffer(), pPacket, size);
    ethernet->write_packet((uint8_t *) pPacket, size);
}

// define this to split full frames into two to illicit an ack from the endpoint
#define SPLIT_OUTPUT

#ifdef SPLIT_OUTPUT
extern "C" void uip_split_output(void);
extern "C" void tcpip_output()
{
    theNetwork->tapdev_send(uip_buf, uip_len);
}
void network_device_send()
{
    uip_split_output();
    //tcpip_output();
}
#else
void network_device_send()
{
    tapdev_send(uip_buf, uip_len);
}
#endif

void Network::handlePacket(void)
{
    if (uip_len > 0) {  /* received packet */
        //printf("handlePacket: %d\n", uip_len);

        if (BUF->type == htons(UIP_ETHTYPE_IP)) { /* IP packet */
            uip_arp_ipin();
            uip_input();
            /* If the above function invocation resulted in data that
                should be sent out on the network, the global variable
                uip_len is set to a value > 0. */

            if (uip_len > 0) {
                uip_arp_out();
                network_device_send();
            }

        } else if (BUF->type == htons(UIP_ETHTYPE_ARP)) { /*ARP packet */
            uip_arp_arpin();
            /* If the above function invocation resulted in data that
                should be sent out on the network, the global variable
                uip_len is set to a value > 0. */
            if (uip_len > 0) {
                tapdev_send(uip_buf, uip_len);  /* ARP ack*/
            }

        } else {
            printf("Unknown ethernet packet type %04X\n", htons(BUF->type));
            uip_len = 0;
        }
    }
}
