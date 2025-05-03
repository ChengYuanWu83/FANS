#include "planner_config.h"

rnl::Nbt::Nbt ()
{
	one_hop = {};
	two_hop = {};
}

void rnl::Nbt::serialize (std::string* dst)
{
    std::stringstream f;    
    for (auto n: one_hop)
        f << n.first << rnl::DELIM_NBTID_POS << n.second.x << rnl::DELIM_NBTPOS << n.second.y << rnl::DELIM_NBTPOS << n.second.z << rnl::DELIM_NBTHOP;
    
    f << rnl::DELIM_NBTMHOP;
    
    for (auto n: two_hop)
        f << n.first << rnl::DELIM_NBTID_POS << n.second.x << rnl::DELIM_NBTPOS << n.second.y << rnl::DELIM_NBTPOS << n.second.z << rnl::DELIM_NBTHOP;
    
    *dst = f.str();
}


void rnl::Nbt::parseSingleNb(std::string msg)
{
	if (msg.size())
	{
		std::string _tok;
        
		/*Get BC ID*/
		_tok = msg.substr (0, msg.find(rnl::DELIM));
		int _id = std::stoi (_tok);
		msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size());
		
		/*Get Pos*/
		_tok          = msg.substr(0, msg.find(rnl::DELIM));
		ns3::Vector3D temp; 
		temp.x       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		temp.y       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		temp.z       = std::stod(_tok.substr(0, _tok.find(rnl::DELIM_POS)));
		_tok.erase(0, _tok.find(rnl::DELIM_POS) + rnl::DELIM_POS.size());
		msg.erase(0, msg.find(rnl::DELIM) + rnl::DELIM.size());
		
		auto it = std::find_if(one_hop.begin(), one_hop.end(),
		[&_id](const std::pair<int, ns3::Vector3D>& _p){return _p.first  == _id; } ); 

		if (one_hop.size() && it->first == _id)
		{
			it->second = temp;
		}
		else
		{
			one_hop.push_back(std::pair<int, ns3::Vector3D>(_id, temp));
		}
	}
}
