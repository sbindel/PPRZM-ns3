/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

/*
 * Copyright (c) 2017 University of Haute Alsace
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Sebastien Bindel  <sebastien.bindel@scientist.fr>
 */

#include "paparazzi-mobility-model.h"
#include <cmath>
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "position-allocator.h"

namespace ns3 {

	NS_LOG_COMPONENT_DEFINE ("Paparazzi");
	NS_OBJECT_ENSURE_REGISTERED (PaparazziMobilityModel);

	TypeId PaparazziMobilityModel::GetTypeId (void){
			static TypeId tid = TypeId ("ns3::PaparazziMobilityModel")
				.SetParent<MobilityModel> ()
				.SetGroupName ("Mobility")
				.AddConstructor<PaparazziMobilityModel> ()
				.AddAttribute ("Bounds",
						"Bounds of the area to cruise.",
						BoxValue (Box (0, 1000.0, 0.0, 1000.0, 0.0, 1000.0)),
						MakeBoxAccessor (&PaparazziMobilityModel::m_bounds),
						MakeBoxChecker ())
				.AddAttribute ("Speed",
						"Velocity of nodes in the Paparazzi mobility model (m/s).",
						DoubleValue (15.0),
						MakeDoubleAccessor (&PaparazziMobilityModel::m_velocity),
						MakeDoubleChecker<double> ())
				.AddAttribute ("Radius",
						"A constant representing the radius of the arch (m).",
						DoubleValue (1.0),
						MakeDoubleAccessor (&PaparazziMobilityModel::m_radius),
						MakeDoubleChecker<double> ());
			return tid;
		}

	PaparazziMobilityModel::PaparazziMobilityModel (){
		m_uniformStartPosition = CreateObject<UniformRandomVariable>();
		m_mode=WAYPOINT;
		m_angle=0;
		m_isdone=false;
		m_first = true;
		m_firstTurn=true;
		m_startComplexMvt=true;
		m_isTurn=false;
		m_firstTurn=true;
		m_event = Simulator::ScheduleNow (&PaparazziMobilityModel::Start, this);
		m_helper.Unpause ();
	}

	void PaparazziMobilityModel::Start (void) {
		Time update;
		if(m_first){
			m_first = false;
			m_current = DoGetPosition();
			m_helper.SetPosition(Vector(0,0,0));
			m_destination=RandomPosition();
			update=MoveWaypoint();
		}
		else {

			if (m_mode==WAYPOINT){
				if((int)CalculateDistance(m_destination, m_helper.GetCurrentPosition())==0){
					update=MoveStayAt();
					m_mode=STAYAT;
					m_destination=GetPosition();
				}
				else{
					update=MoveWaypoint();
				}
			}
			else if (m_mode==STAYAT){
				if(m_isdone){
					m_mode=OVAL;
					m_origin=GetPosition();
					update=MoveOval();
				}
				else {
					update=MoveStayAt();
				}
			}
			else if (m_mode==OVAL){
				if((int)CalculateDistance(m_origin, m_helper.GetCurrentPosition())==0){
					m_mode=EIGHT;
					m_origin=GetPosition();
					m_startComplexMvt=true;
					update=MoveEight();
				}
				else{
					update=MoveOval();
				}
			}
			else if (m_mode==EIGHT){
				update=MoveEight();
			}
		}
		m_current = DoGetPosition();
		m_helper.Update ();
		m_helper.Unpause ();
		DoWalk (update);
	}

void PaparazziMobilityModel::DoWalk (Time delayLeft){
		m_helper.UpdateWithBounds (m_bounds);
		if (m_bounds.IsInside (m_next)){
			m_event = Simulator::Schedule (delayLeft, &PaparazziMobilityModel::Start, this);
		}
		else{
			//nextPosition = m_bounds.CalculateIntersection (position, speed);
			//Time delay = Seconds ((nextPosition.x - position.x) / speed.x);
			//m_event = Simulator::Schedule (delay, &PaparazziMobilityModel::Rebound, this,
			//                               delayLeft - delay);
		}
		NotifyCourseChange ();
	}

	Vector3D PaparazziMobilityModel::RandomPosition(void) const{
		Vector3D posi(0,0,0);
		posi.x = m_uniformStartPosition->GetValue(m_bounds.xMax-m_radius, m_bounds.xMin+m_radius);
		posi.y = m_uniformStartPosition->GetValue(m_bounds.yMax-m_radius, m_bounds.yMin+m_radius);
		if (m_bounds.zMax>0)
			posi.z = m_uniformStartPosition->GetValue(m_bounds.zMax-m_radius, m_bounds.zMin+m_radius);
		return posi;
	}

	bool PaparazziMobilityModel::CircularMouvement(const int& endDegree, const int& angle){
		m_current=m_helper.GetCurrentPosition();
		m_angle+=angle;
		double current_angle = M_PI*m_angle/180;
		Vector position (0,0,0);
		position.y = m_current.y + std::sin(current_angle);
		position.x = m_current.x + std::cos(current_angle);
		m_helper.SetPosition(Vector (position.x, position.y, position.z));
		return (m_angle==endDegree)?true:false;
	}

	Time PaparazziMobilityModel::MoveWaypoint(){
		m_current=m_helper.GetCurrentPosition();
		double dx = (m_destination.x - m_current.x);
		double dy = (m_destination.y - m_current.y);
		double dz = (m_destination.z - m_current.z);
		double k = m_velocity / std::sqrt (dx*dx + dy*dy + dz*dz);
		m_helper.SetVelocity (Vector (k*dx, k*dy, k*dz));
		return Seconds (CalculateDistance (m_destination, m_current) / m_velocity);
	}

	Time PaparazziMobilityModel::MoveStayAt(){
		m_isdone=CircularMouvement(360,ANGLE);
		return Seconds(0.1);
	}

	Time PaparazziMobilityModel::MoveOval(){
		Time temps;
		if(m_startComplexMvt){
			m_destination=RandomPosition();
			temps=MoveWaypoint();
			m_isTurn=true;
			m_startComplexMvt=false;
			m_angle=ANGLE;
		}
		else {
			if (!m_isTurn){
				if(m_helper.GetCurrentPosition().x<=m_origin.x)
					m_destination.x=m_origin.x - 4;
				else
					m_destination.x=m_origin.x + 4;

				if(m_helper.GetCurrentPosition().y<=m_origin.y)
					m_destination.y=m_origin.y + 6;
				else
					m_destination.y=m_origin.y - 6;
				m_destination.z=m_origin.z;
				
				
				temps=MoveWaypoint();
				m_isTurn=true;
			}
			else {
				m_isTurn=!CircularMouvement(90,ANGLE);
				temps=Seconds(0.1);
			}
		}
		return temps;
	}

	Time PaparazziMobilityModel::MoveEight(){
		Time temps;
		if(m_startComplexMvt){
			m_destination=RandomPosition();
			temps=MoveWaypoint();
			m_isTurn=true;
			m_startComplexMvt=false;
			m_angle=ANGLE;
		}
		else {

			if(!m_isTurn){
				if(m_helper.GetCurrentPosition().x>m_origin.x)
					m_destination.x=m_origin.x - 4;
				else
					m_destination.x=m_origin.x + 4;

				if(m_helper.GetCurrentPosition().y>m_origin.y)
					m_destination.y=m_origin.y + 6;
				else
					m_destination.y=m_origin.y - 6;
				m_destination.z=m_origin.z;
				
				temps=MoveWaypoint();
				m_angle=270;
				m_isTurn=true;
			}
			else{
				if(m_firstTurn){
					m_first=false;
					m_isTurn=!CircularMouvement(90,ANGLE);
				}
				else{
					m_first=true;
					m_isTurn=!CircularMouvement(270,ANGLE); //TODO allow to choice circular mouvement
				}
				temps=Seconds(0.1);
			}
		}
		return temps;
	}

	void PaparazziMobilityModel::DoDispose (void){
		// chain up
		MobilityModel::DoDispose ();
	}

	Vector PaparazziMobilityModel::DoGetPosition (void) const{
		m_helper.UpdateWithBounds (m_bounds);
		return m_helper.GetCurrentPosition ();
	}

	void PaparazziMobilityModel::DoSetPosition (const Vector &position){
		NS_ASSERT (m_bounds.IsInside (position));
		m_helper.SetPosition (position);
		Simulator::Remove (m_event);
		m_event = Simulator::ScheduleNow (&PaparazziMobilityModel::Start, this);
	}

	Vector PaparazziMobilityModel::DoGetVelocity (void) const{
		return m_helper.GetVelocity ();
	}


	int64_t PaparazziMobilityModel::DoAssignStreams (int64_t stream){
		m_uniformStartPosition->SetStream(stream);
		return 1;
	}
}

