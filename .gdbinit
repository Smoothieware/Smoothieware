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

document heapwalk
Walks the heap and dumps each chunk encountered.
It will also lists the line and source filename from where the chunk was
allocated if not a freed chunk. Requires that HEAP_WALK be set to a value of 1
in the Smoothie makefile.
end




define heapsize
    if ($argc > 0)
        set var $heap_base=(unsigned int)$arg0
    else
        set var $heap_base=(unsigned int)__smoothieHeapBase
    end
    printf "heap size: %d bytes\n", ('_sbrk::heap' - $heap_base)
end

document heapsize
Displays the current heap size.
Can provide an optional argument specifying the location of the base address
for the heap. This isn't required if you have HEAP_WALK enabled in the makefile
but if that features isn't enabled, you will want to run 
"maintenance info section .heap" to determine this base address and then
pass it as an argument to this comand.
end



define stacksize
    printf "stack size: %d bytes\n", 0x10008000 - (unsigned int)$sp 
end

document stacksize
Displays the current stack size.
end



define freespace
    printf "free space: %d bytes\n", (unsigned int)$sp - '_sbrk::heap'
end

document freespace
Displays the free space.

This is the amount of space between the heap and stack that is currently
unused.
end



define maxstacksize
    set var $fill_curr=(unsigned int*)'_sbrk::heap'
    while ($fill_curr < $sp && *$fill_curr == 0xdeadbeef)
        set var $fill_curr = $fill_curr + 1
    end

    if ($fill_curr == '_sbrk::heap')
        printf "No free space between heap and stack detected!\n"
    else
        printf "maximum stack size: %d bytes\n", 0x10008000 - (unsigned int)$fill_curr
    end
end

document maxstacksize
Displays the maximum stack amount of stack used.
This can take awhile to run as it walks the area between the top of heap and
the current top of stack to look for where initial fill values have been
overwritten by stack writes.
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

document crashdump
Dumps a full crash dump to gdb.txt
end
