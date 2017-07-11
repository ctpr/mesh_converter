/*
 * main.cpp
 *
 *  Created on: Nov 23, 2011
 *      Author: CR
 */

///////////////////
//c++ includes
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
///////////////////

///////////////////
//external includes
//#include <rapidxml.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
///////////////////

///////////////////
//internal includes
#include <MeshConverter.hpp>
///////////////////

///////////////////
//const values
//const rapidxml::xml_document<> QV_CONFIG_DEFAULT;
const int MAYOR_VERSION = 0;
const int MINOR_VERSION = 1;
///////////////////

void printHelp()
{
    printf("========================\n");
    printf("Mesh converter v%i.%i alpha (Author: Christoph Russ, License: GPL)\n", MAYOR_VERSION, MINOR_VERSION);
    printf("Please use the following input arguments:\n");
    printf(" -h               // display this help text\n");
    printf(" -p <INPUT_FILE.OBJ> <OUTPUT_FILE.POLY> // convert a OBJ file to a POLY\n");
    printf("\n");
}


int initConfig(int argc, char **argv)
{
	// TODO: add QVParameters &params

    bool quitOnExit = false;
    bool parseOn = false;
    bool loadGeometry = false;
    bool exportGeometry = false;
    bool performAction = false;

    const char * inputFile = "./input";
    const char * outputFile = "./output";

    const char * actions = "non";

    //rapidxml::xml_document<> configDoc;

    for (int i = 0; i < argc; ++i)
    {
        switch (argv[i][0])
        {
            case '-':
                switch (argv[i][1])
                {
                    case 'h': //help
                        //print help & exit
                        printHelp();
                        exit(1);
                        break;
                    case 'c': //config
                        //read user provided config file
                        if (i+1 < argc)
                        {
                            ++i;
                            configFile = argv[i];
                        }
                        break;
                    case 'p': //parse
                        if (i+2 < argc)
                        {
                            ++i;
                            inputFile = argv[i];
                            ++i;
                            outputFile = argv[i];

                            parseOn = true;
                        }
                        break;
                    case 's': //subdivide
                        if (i+2 < argc)
                        {
                            ++i;
                            inputFile = argv[i];
                            ++i;
                            outputFile = argv[i];

                            subdivisionOn = true;
                        }
                        break;
                    case 'l':
                    case 'i': //input load (in addition to the config file, we want to simply load individual models
                        if (i+1 < argc)
                        {
                            ++i;
                            inputFile = argv[i];

                            loadGeometry = true;
                        }
                        break;
                    case 'o':
                    case 'e': //output export (in addition to the config file, we want to simply load individual models
                        if (i+1 < argc)
                        {
                            ++i;
                            outputFile = argv[i];

                            exportGeometry = true;
                        }
                                            break;
                    case 'a': //actions (for script based usage of certain actions)
                        if (i+1 < argc)
                        {
                            ++i;
                            actions = argv[i];

                            performAction = true;
                        }
                        break;
                    default:
                        break;
                }
                break;
            default: //argument
                break;
        }
    }

    std::vector<std::string> fileContents;
    if (QVConfig::ReadFile(configFile, fileContents, true) == 0)
        QVLogger::LogInfo("Using default initializers.\n");

    // TODO: QVConfig::ParseConfig(fileContents, params);


    if (parseOn || subdivisionOn || loadGeometry || exportGeometry || performAction)
    //if (loadGeometry || exportGeometry || performAction)
    {
        //check for multiple input files separate by ',' or ';' or ':' ...
        std::string tmp_string = inputFile;
        std::vector<std::string> strs;
        boost::split(strs, tmp_string, boost::is_any_of(",;: "), boost::token_compress_on);

        QVModelLoader * modelLoader = QVModelLoader::Instance();

        for (size_t i=0; i<strs.size(); ++i)
        {
            inputFile = strs.at(i).c_str();
            modelLoader->importData(inputFile, outputFile, parseOn, subdivisionOn, loadGeometry);
            //modelLoader->importData(inputFile);
        }

        if (performAction)
        {
            tmp_string = actions;
            strs.clear();
            boost::split(strs, tmp_string, boost::is_any_of(",;: "), boost::token_compress_on);

            QVProcessManager * procMgr = QVProcessManager::Instance();

            //perform all actions in this list - one after the other
            //this will allow us to create simple workflows / pipelines for mesh import - processing - export
            for (size_t i=0; i<strs.size(); ++i)
            {
                procMgr->addToQueue(strs.at(i));
            }
        }

        //export geometry straight away here (after processing or whatever might have been done)
        if (exportGeometry)
        {
            QVModelExporter * modelExporter = QVModelExporter::Instance();

            tmp_string = outputFile;
            strs.clear();
            boost::split(strs, tmp_string, boost::is_any_of(",;: "), boost::token_compress_on);

            for (size_t i=0; i<strs.size(); ++i)
            {
                outputFile = strs.at(i).c_str();
                modelExporter->exportData(outputFile);
            }
        }

        if (quitOnExit || parseOn || subdivisionOn)
        {
            //Q: shall we really exit the application here (after parsing is complete) ?
            exit(1);
        }
    }

    return 1;
}

void testing()
{
    MeshConverter * meshConverter = new MeshConverter();

    meshConverter->testPCL();

    delete meshConverter;
    meshConverter = NULL;
}


int main(int argc, char **argv)
{
    //initConfig(argc, argv);

    // CURRENTLY ONLY RUNNING SOME TEST CODE
    testing();

    return 1;
}
