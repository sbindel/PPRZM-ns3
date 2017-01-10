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

#ifndef PAPARAZZI_MOBILITY_MODEL_H
#define PAPARAZZI_MOBILITY_MODEL_H

#define ANGLE 10

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/rectangle.h"
#include "ns3/mobility-model.h"
#include "ns3/constant-velocity-helper.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

	class PaparazziMobilityModel : public MobilityModel {
		
		public:
			static TypeId GetTypeId (void);
			PaparazziMobilityModel ();

			enum Mode {
				STAYAT,
				WAYPOINT,
				EIGHT,
				SCAN,
				OVAL
			};

		private:
			void Start (void);
			void DoWalk (Time timeLeft);
			Vector3D RandomPosition() const;
			bool CircularMouvement(const int& endDregree, const int& angle);
			Time MoveWaypoint(void);
			Time MoveStayAt(void);
			Time MoveOval(void);
			Time MoveEight(void);
			virtual void DoDispose (void);
			virtual Vector DoGetPosition (void) const;
			virtual void DoSetPosition (const Vector &position);
			virtual Vector DoGetVelocity (void) const;
			virtual int64_t DoAssignStreams (int64_t);

			ConstantVelocityHelper m_helper;
			enum Mode m_mode;
			EventId m_event;
			Box m_bounds;
			Vector m_current;
			Vector m_next;
			double m_velocity;
			double m_radius;
			double m_angle;
			bool m_first;
			bool m_isdone;
			bool m_startComplexMvt;
			bool m_isTurn;
			bool m_firstTurn;
			Vector3D m_destination;
			Vector3D m_origin;
			Ptr<UniformRandomVariable> m_uniformStartPosition;
	}; 
}
#endif /* PAPARAZZI_MOBILITY_MODEL_H */
