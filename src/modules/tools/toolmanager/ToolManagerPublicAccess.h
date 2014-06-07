#ifndef __TOOLMANAGERPUBLICACCESS_H
#define __TOOLMANAGERPUBLICACCESS_H

// addresses used for public data access
#define tool_manager_checksum             CHECKSUM("tool_manager")
#define current_tool_name_checksum        CHECKSUM("current_tool_name")

struct pad_toolmanager {
    uint16_t current_tool_name;
};

#endif // __TOOLMANAGERPUBLICACCESS_H

