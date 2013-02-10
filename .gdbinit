define heapwalk
    set var $chunk_curr=(unsigned int)__smoothieHeapBase
    set var $chunk_number=1
    while ($chunk_curr < '_sbrk::heap')
        set var $chunk_size=*(unsigned int*)($chunk_curr+4) & ~1
        set var $chunk_next=$chunk_curr + $chunk_size
        set var $chunk_inuse=*(unsigned int*)($chunk_next+4) & 1
        set var $chunk_tag=*(unsigned int*)$chunk_next
        printf "Allocation: %u  Address: 0x%08X  Size:%u  ", $chunk_number, $chunk_curr+8, $chunk_size-4
        if ($chunk_inuse)
            info line *($chunk_tag)
        else
            printf "FREE CHUNK\n"
        end
        set var $chunk_curr=$chunk_next
        set var $chunk_number=$chunk_number+1
    end
end

define crashdump
    set pagination off
    set logging on
    bt
    list
    disass
    set var $ptr=0x10000000
    while $ptr < 0x10008000
    x/4wa $ptr
    set var $ptr+=16
    end
    info registers
    set logging off
    set pagination on
end
