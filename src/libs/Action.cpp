/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include <math.h>
#include <string>

#include "mri.h"

#include "Action.h"
#include "Planner.h"
#include "Conveyor.h"

using std::string;
#include <vector>

Action::Action()
{
    first_data = NULL;
    gc_data = NULL;

    block_data = NULL;
}

Action* Action::add_data(ActionData* data)
{
    printf("Action::add_data(%p, %p)\n", this, data);

    data->next = first_data;
    first_data = data;

    data->action = this;

    return this;
}

Action* Action::remove_data(ActionData* data)
{
    printf("Action::remove_data(%p, %p)\n", this, data);

    if (!data)
        return this;

    if (!first_data)
        return this;

    else if (first_data == data)
    {
        first_data = data->next;
    }
    else
    {
        ActionData* d;
        for (d = first_data;
            d && d->next && (d->next != data);
            d = d->next);

        if (d->next == data)
        {
            d->next = data->next;
        }
    }
    data->next = gc_data;
    gc_data = data;

    // if action is complete, start next action
    if (first_data == NULL)
        conveyor->start_next_action();

    return this;
}

Action* Action::invoke()
{
    ActionData* data = first_data;
    ActionData* temp;

    printf("Action::invoke(%p)\n", this);

    while (data)
    {
        printf("\tData: %p\n", data);

        // we use a temp register because on_action_invoke might remove the ActionData straight away,
        // in which case data->next points to an item in the garbage collection queue and we'd get confused
        temp = data;
        data = data->next;
        printf("\tinvoke(%p, owner:%p)\n", temp, temp->owner);
        temp->owner->on_action_invoke(temp);
    }

    return this;
}

Action* Action::clean()
{
    ActionData* d;

    while ((d = gc_data))
    {
        gc_data = gc_data->next;

        delete d;
    }

    if (first_data)
    {
        // THIS SHOULD NEVER EXECUTE DURING NORMAL OPERATION
        printf("warning: Action with current data cleaned!\n");
        __debugbreak();

        while ((d = first_data))
        {

            first_data = first_data->next;

            delete d;
        }
    }

    // block should have been deleted in one of the two loops above since it's just a shortcut pointer and not a separate item
    // TODO: ensure that this block has been deleted before we nuke our reference to it
    block_data = NULL;

    return this;
}
