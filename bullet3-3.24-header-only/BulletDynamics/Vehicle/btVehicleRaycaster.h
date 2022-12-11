/*
 * Copyright (c) 2005 Erwin Coumans http://bulletphysics.org
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_VEHICLE_RAYCASTER_H
#define BT_VEHICLE_RAYCASTER_H

#include "btApi.h"

#include "LinearMath/btVector3.h"

/// btVehicleRaycaster is provides interface for between vehicle simulation and raycasting
struct BT_API btVehicleRaycaster
{
	virtual ~btVehicleRaycaster()
	{
	}
	struct BT_API btVehicleRaycasterResult
	{
		btVehicleRaycasterResult() : m_distFraction(btScalar(-1.)){};
		btVector3 m_hitPointInWorld;
		btVector3 m_hitNormalInWorld;
		btScalar m_distFraction;
	};

	virtual void* castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result) = 0;
};

#endif  //BT_VEHICLE_RAYCASTER_H