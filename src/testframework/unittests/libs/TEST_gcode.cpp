#include "utils.h"

#include "Gcode.h"

#include <vector>
#include <stdio.h>
#include <string.h>

#include "easyunit/test.h"

TEST(GCodeTest,subcode)
{
    Gcode gc1("G32 X1.2 Y2.3", nullptr);

    ASSERT_TRUE(gc1.has_g);
    ASSERT_TRUE(!gc1.has_m);
    ASSERT_EQUALS_V(32, gc1.g);
    ASSERT_EQUALS_V(0, gc1.subcode);
    ASSERT_EQUALS_V(2, gc1.get_num_args());
    ASSERT_TRUE(gc1.has_letter('X'));
    ASSERT_TRUE(gc1.has_letter('Y'));
    ASSERT_EQUALS_DELTA_V(1.2, gc1.get_value('X'), 0.001);
    ASSERT_EQUALS_DELTA_V(2.3, gc1.get_value('Y'), 0.001);

    Gcode gc2("G32.2 X1.2 Y2.3", nullptr);

    ASSERT_TRUE(gc2.has_g);
    ASSERT_TRUE(!gc2.has_m);
    ASSERT_EQUALS_V(32, gc2.g);
    ASSERT_EQUALS_V(2, gc2.subcode);
    ASSERT_EQUALS_V(2, gc2.get_num_args());
    ASSERT_TRUE(gc2.has_letter('X'));
    ASSERT_TRUE(gc2.has_letter('Y'));
    ASSERT_EQUALS_DELTA_V(1.2, gc2.get_value('X'), 0.001);
    ASSERT_EQUALS_DELTA_V(2.3, gc2.get_value('Y'), 0.001);

    // test equals
    Gcode gc3("", nullptr);
    gc3= gc2;
    ASSERT_TRUE(gc3.has_g);
    ASSERT_TRUE(!gc3.has_m);
    ASSERT_EQUALS_V(32, gc3.g);
    ASSERT_EQUALS_V(2, gc3.subcode);
    ASSERT_EQUALS_V(2, gc3.get_num_args());
    ASSERT_TRUE(gc3.has_letter('X'));
    ASSERT_TRUE(gc3.has_letter('Y'));
    ASSERT_EQUALS_DELTA_V(1.2, gc3.get_value('X'), 0.001);
    ASSERT_EQUALS_DELTA_V(2.3, gc3.get_value('Y'), 0.001);

    // test copy ctor
    Gcode gc4(gc2);
    ASSERT_TRUE(gc4.has_g);
    ASSERT_TRUE(!gc4.has_m);
    ASSERT_EQUALS_V(32, gc4.g);
    ASSERT_EQUALS_V(2, gc4.subcode);
    ASSERT_EQUALS_V(2, gc4.get_num_args());
    ASSERT_TRUE(gc4.has_letter('X'));
    ASSERT_TRUE(gc4.has_letter('Y'));
    ASSERT_EQUALS_DELTA_V(1.2, gc4.get_value('X'), 0.001);
    ASSERT_EQUALS_DELTA_V(2.3, gc4.get_value('Y'), 0.001);

}
