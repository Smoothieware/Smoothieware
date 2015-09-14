#include "utils.h"

#include <vector>
#include <stdio.h>

#include "easyunit/test.h"

TEST(UtilsTest,split)
{
    const char *s= "one two three";
    std::vector<std::string> v= split(s, ' ');
    ASSERT_TRUE(v.size() == 3);
    ASSERT_TRUE(v[0] == "one");
    ASSERT_TRUE(v[1] == "two");
    ASSERT_TRUE(v[2] == "three");
}

TEST(UtilsTest,split_empty_string)
{
    const char *s= "";
    std::vector<std::string> v= split(s, ' ');

     ASSERT_TRUE(v.size() == 1);
     ASSERT_TRUE(v[0].empty());
     ASSERT_TRUE(v[0] == "");
}

TEST(UtilsTest,parse_number_list)
{
    const char *s= "1.1,2.2,3.3";
    std::vector<float> v= parse_number_list(s);
    ASSERT_TRUE(v.size() == 3);
    ASSERT_TRUE(v[0] == 1.1F);
    ASSERT_TRUE(v[1] == 2.2F);
    ASSERT_TRUE(v[2] == 3.3F);
}

TEST(UtilsTest,append_parameters)
{
    std::string str;

    str= append_parameters({{'X', 1}, {'Y', 2}, {'Z', 3}});
    ASSERT_EQUALS(str, "X1 Y2 Z3 ");
}
