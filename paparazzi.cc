#include "ns3/core-module.h"
#include "ns3/netanim-module.h"
#include "ns3/paparazzi-mobility-model.h"
#include "ns3/stats-module.h"
#include "ns3/mobility-module.h"
#include "ns3/simulator.h"
#include <iostream>
#include <sstream>

using namespace ns3;

int main (int argc, char *argv[]) {
	
	CommandLine cmd;
	cmd.Parse(argc,argv);

	NodeContainer c;
	c.Create (1);

	MobilityHelper mobility;
	mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
			"X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=200]"),
			"Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=200]"),
			"Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"));

	mobility.SetMobilityModel ("ns3::PaparazziMobilityModel",
			"Radius", StringValue ("10"),
			"Bounds", BoxValue (Box (0, 200, 0, 200, 0, 0)));
	mobility.InstallAll ();

	AnimationInterface anim ("paparazzi.xml");

	Simulator::Stop (Seconds (60));

	Simulator::Run ();

	Simulator::Destroy ();
	return 0;
}
