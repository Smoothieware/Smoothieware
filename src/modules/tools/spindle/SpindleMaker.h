/*
 *       this file is part of smoothie (http://smoothieware.org/). the motion control part is heavily based on grbl (https://github.com/simen/grbl).
 *             smoothie is free software: you can redistribute it and/or modify it under the terms of the gnu general public license as published by the free software foundation, either version 3 of the license, or (at your option) any later version.
 *                   smoothie is distributed in the hope that it will be useful, but without any warranty; without even the implied warranty of merchantability or fitness for a particular purpose. see the gnu general public license for more details.
 *                         you should have received a copy of the gnu general public license along with smoothie. if not, see <http://www.gnu.org/licenses/>.
 *                         */

#ifndef SPINDLEMAKER_H
#define SPINDLERMAKER_H

class SpindleControl;

class SpindleMaker {
    public:
        SpindleMaker() {};
        virtual ~SpindleMaker() {};
        
        void load_spindle();

    private:
        SpindleControl* spindle;
            
};

#endif
