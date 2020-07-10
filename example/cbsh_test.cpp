/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2019
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/

#include <CBSH2/map_loader.h>
#include <CBSH2/agents_loader.h>
#include <CBSH2/ICBSSearch.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <chrono>
#include <fstream>
#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

int main(int argc, char** argv) 
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("input,i", po::value<std::string>()->required(), "input file for map")
		("PC,p", po::value<bool>()->default_value(true), "conflict prioritization")
		("heuristics,h", po::value<std::string>()->default_value("NONE"), "heuristics for the high-level search (NONE, CG, DG, WDG)")
		("cutoffTime,t", po::value<int>()->default_value(7200), "cutoff time (seconds)")
		("MaxMDDs", po::value<int>(), "maximum number of MDDs saved for each pair of agents")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		("rectangleReasoning,r", po::value<bool>()->default_value(false), "Using rectangle reasoning")
		("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("warehouseWidth,b", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instacnes")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));
	std::string inputFile = vm["input"].as<string>();
	
	YAML::Node config = YAML::LoadFile(inputFile);

	std::vector<std::pair<int, int> > obstacles;
	std::vector<std::pair<int, int> > goals;
	std::vector<std::pair<int, int> > starts;

	const auto& dim = config["map"]["dimensions"];
	int dimx = dim[0].as<int>();
	int dimy = dim[1].as<int>();

	for (const auto& node : config["map"]["obstacles"]) {
	obstacles.emplace_back(std::make_pair(node[0].as<int>(), node[1].as<int>()));
	}

	for (const auto& node : config["agents"]) {
	const auto& start = node["start"];
	const auto& goal = node["goal"];
	starts.emplace_back(std::make_pair(start[0].as<int>(), start[1].as<int>()));
	// std::cout << "s: " << startStates.back() << std::endl;
	goals.emplace_back(std::make_pair(goal[0].as<int>(), goal[1].as<int>()));
	}

	// read the map file and construct its two-dim array
	CBSH::MapLoader ml(obstacles, starts, goals, dimx, dimy, obstacles.size());

	// read agents' start and goal locations
	CBSH::AgentsLoader al(starts, goals);
 
	srand(vm["seed"].as<int>());

	CBSH::heuristics_type h;
	if (vm["heuristics"].as<string>() == "NONE")
		h = CBSH::heuristics_type::NONE;
	else if (vm["heuristics"].as<string>() == "CG")
		h = CBSH::heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = CBSH::heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = CBSH::heuristics_type::WDG;
	else
	{
		std::cout <<"WRONG HEURISTICS NAME!" << std::endl;
		return -1;
	}
	CBSH::ICBSSearch icbs(ml, al, 1.0, h, vm["PC"].as<bool>(), vm["rectangleReasoning"].as<bool>(), 
		vm["cutoffTime"].as<int>() * 1000, vm["screen"].as<int>());
	if (vm.count("MaxMDDs"))
		icbs.max_num_of_mdds = vm["MaxMDDs"].as<int>();
	bool res;

	auto cbsh_start = std::chrono::system_clock::now();
	res = icbs.runICBSSearch();
	auto cbsh_end = std::chrono::system_clock::now();
	auto cbsh_time = std::chrono::duration<double>(cbsh_end - cbsh_start).count();	
	
	//icbs.printResults();
	//icbs.printPaths();
	
	std::pair<int, std::vector< std::vector< std::pair<int, int> > > > solution = icbs.getPaths();
	
	if (res) {
		std::cout << "Planning successful! " << std::endl;
		std::cout<<"Optimal Cost :: "<<solution.first<<std::endl; 
		std::ofstream out("../example/output_cbs-h.yaml");
		out << "statistics:" << std::endl;
		out << "  cost: " << solution.first << std::endl;
		out << "  runtime: " << cbsh_time << std::endl;
		out << "schedule:" << std::endl;

		int count = 0;

		for(auto it=solution.second.begin(); it!=solution.second.end();++it){
			out << "  agent" << count << ":" << std::endl;
			std::vector<std::pair<int, int> > output;
			output = *it;
			for(int i=0; i<output.size(); i++){
				out << "    - x: " << output[i].first << std::endl
					<< "      y: " << output[i].second << std::endl
					<< "      t: " << i << std::endl;
			}
			count++;
		}
	} else {
		std::cout << "Planning NOT successful!" << std::endl;
	}

	icbs.clearSearchEngines();
	return 0;

}
