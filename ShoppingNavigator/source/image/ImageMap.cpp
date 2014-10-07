/**********************************************************************************
 *  The MIT License (MIT)                                                        *
 *                                                                                *
 *  Copyright (c) 2014 Carnegie Mellon University                                 *
 *                                                                                *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy  *
 *  of this software and associated documentation files (the "Software"), to deal *
 *  in the Software without restriction, including without limitation the rights  *
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 *  copies of the Software, and to permit persons to whom the Software is         *
 *  furnished to do so, subject to the following conditions:                      *
 *                                                                                *
 *  The above copyright notice and this permission notice shall be included in    *
 *  all copies or substantial portions of the Software.                           *
 *                                                                                *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     *
 *  THE SOFTWARE.                                                                 *
 **********************************************************************************/
/**
 * @file   ImageMap.cpp
 * @brief  Lookup image locations from a pre-built map
 * @author M. George
 */

#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/lexical_cast.hpp>

#include "ImageMap.h"
bool ImageMap::load(std::string mapfile)
{
    // Map data is a formatted string in a .csv file
    std::ifstream map(mapfile.c_str());
    std::string place;
    
    // We will deconstruct the string into a filename and a location
    std::string imagefile;
    std::string coordinate;
    BASIL::CartesianVector imagelocation;

    while(std::getline(map,place))
    {
        std::stringstream placedata(place);
        // First entry is filename
        std::getline(placedata,imagefile,',');
        std::getline(placedata,coordinate,',');
        imagelocation(BASIL::NORTH) = boost::lexical_cast<double>(coordinate);
        std::getline(placedata,coordinate,',');
        imagelocation(BASIL::EAST) = boost::lexical_cast<double>(coordinate);
        std::getline(placedata,coordinate,',');
        imagelocation(BASIL::DOWN) = boost::lexical_cast<double>(coordinate);
        // Save it in the stl map
        Map[imagefile] = imagelocation;
    }
    return true;
 }

