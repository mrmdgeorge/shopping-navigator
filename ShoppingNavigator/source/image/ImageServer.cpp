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
 * @file   ImageServer.cpp
 * @brief  Grab images from a Pt. Grey camera and transmit them to FABMAP via MOOS
 * @author M. George
 */

#include <iostream>
#include <string>
#include <stdio.h>
#include <unistd.h>

#include "basil/messages/sensors/cplacerecognitionmsg.h"

#include "FABMAP.h"
#include "ImageMap.h"
#include "CameraCapture.h"
#include "UdpServer.h"

int main(int argc, char * argv[])
{
    std::string mapPath;
    std::string cwd = get_current_dir_name();
    
    if (argc > 1)
    {
        mapPath = std::string(argv[1]);
    }
    else
    {
        std::cerr << "Error: Map file must be passed as first argument." << std::endl;
    }
    
    std::cout << "Loading map file: " << mapPath << std::endl;
    ImageMap map;
    map.load(mapPath + "map.csv");
    
    //Create a client
    //The parameters are the IP address and port of the MOOSDB.
    CFABMAPClient FABMAPEngine;
    FABMAPEngine.Open("localhost",9000);
    
    // Send the map to fabmap
    std::map<std::string,BASIL::CartesianVector>::iterator i;
    for(i = map.Map.begin(); i != map.Map.end(); ++i)
    {
        std::cout << "Loading Map Image: " << mapPath + i->first << std::endl;
          
        //Supply images by providing their full path
        //The FABMAP engine computes how likely each of the places in the map 
        //was to have generated the given image.
        //The image is also added to the map so that it can be recognised in future.
        //
        //The return value is the ID of the "place" added. This value can be used
        //in the GetLocationProbabiity method. 
        int ID = FABMAPEngine.ProcessImage(mapPath + i->first);
    }
    
    // Set up a camera capture interface
    CameraCapture capture;
    capture.setMapSize(map.Map.size());
    int capturecount = map.Map.size();
    char livefilename[10];

    // Set up a udp server to send data to the filter
    UdpServer udpserver("127.0.0.1",1555);
    udpserver.connect();

    // Set up a template of a BASIL landmark message type for transmitting
    // location matches to the inertial filter.
    BASIL::PLACE_RECOGNITION::OXFORD::FABMAP::PlaceRecognitionMsg matchedPlace;
    matchedPlace.header.sync[0] = 'C';
    matchedPlace.header.sync[1] = 'M';
    matchedPlace.header.sync[2] = 'U';
    matchedPlace.header.headerSize = sizeof(BASIL::BasilHeader);
    matchedPlace.header.hardwareId = 0;
    matchedPlace.header.deviceId = 0;
    matchedPlace.header.messageType = static_cast<uint8_t>(BASIL::MESSAGE::PLACE_RECOGNITION);
    matchedPlace.header.messageVersion = static_cast<uint8_t>(BASIL::PLACE_RECOGNITION::OXFORD_FABMAP);
    matchedPlace.header.dataSize = sizeof(BASIL::PLACE_RECOGNITION::OXFORD::FABMAP::PlaceRecognitionMsg) - sizeof(BASIL::BasilHeader) - sizeof(BASIL::BasilCrc);
    matchedPlace.header.reserved = 0;
    matchedPlace.crc = 0;
    matchedPlace.header.time.setTime(BASIL::TIME::FREE_RUNNING,0,0.0);
    
    while (true)
    {
        bool captured = capture.getFrame();
        if (captured)
        {
            sprintf(livefilename,"%.6u",capture.savedCount+capture.mapSize-1);
            int ID = FABMAPEngine.ProcessImage(cwd + "/" + livefilename + ".jpg");
            
            //Get the most probable place
            //The possibilities are the places in the map
            //or the special NEW PLACE value, meaning that the image 
            //was most likely to have been generated by a place not already visited.
            //If this is the case sFileName will be set to the value "NEW PLACE"
            double dfProb;
            unsigned int nPlace;
            std::string sPlaceName;
            std::string sFileName;
            std::string sFileNameStripped;
            if(FABMAPEngine.GetMostProbablePlace(nPlace,sFileName, dfProb))
            {
                // If the most probably place was in our map with high probability, send it to the filter
                sFileNameStripped = sFileName;
                sFileNameStripped.erase(sFileNameStripped.begin(),sFileNameStripped.end()-10);
                std::map<std::string,BASIL::CartesianVector>::iterator i = map.Map.find(sFileNameStripped);
                if (i != map.Map.end() && dfProb > 0.99)
                {
                    matchedPlace.payload.most_similar_place = nPlace;
                    matchedPlace.payload.location[0] = map.Map[sFileNameStripped](0);
                    matchedPlace.payload.location[1] = map.Map[sFileNameStripped](1);
                    matchedPlace.payload.location[2] = map.Map[sFileNameStripped](2);
                    matchedPlace.payload.probability = dfProb;

                    std::cout<<"Found place in map.  Currently at location: "<< nPlace <<" which corresponds to " << sFileNameStripped << std::endl;
                    std::cout<<"Estimated probability of match is "<< dfProb << std::endl << std::endl;
                    std::cout<<"Current coordinates are: " << matchedPlace.payload.location[0] << ", " << matchedPlace.payload.location[1] << ", " << matchedPlace.payload.location[2] << std::endl;
                
                    // We need to add it to the metric map because FabMap will recognize the most recent place before the original map place                    
                    //std::cout << "Adding new place: " << std::string(livefilename) + ".jpg"
                    map.Map[std::string(livefilename) + ".jpg"] =  map.Map[sFileNameStripped];                    
                    
                    // Send information to the positioning system
                    udpserver.send(reinterpret_cast<unsigned char*>(&matchedPlace),sizeof(sizeof(BASIL::PLACE_RECOGNITION::OXFORD::FABMAP::PlaceRecognitionMsg)));
                }
                else
                {
                    std::cout<<"Most Probable Place Is Not In Map." << std::endl;
                }
            }
            else
            {
                std::cout<<"Something is wrong...!"<<std::endl;
            }
        }
    }


}
