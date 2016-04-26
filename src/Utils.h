//
//  ofxCSGUtils.h
//
//  Created by lars berg on 3/3/15.
//

#pragma once

#include "ofMain.h"

namespace states
{
    enum General_state
    {
        GENERAL_STATE_CALIBRATION = 0,
        GENERAL_STATE_SANDBOX = 1
    };
    enum Calibration_state
    {
        CALIBRATION_STATE_ROI_DETERMINATION = 0,
        CALIBRATION_STATE_PROJ_KINECT_CALIBRATION = 1,
        CALIBRATION_STATE_CALIBRATION_TEST = 2
    };
    enum ROI_calibration_state
    {
        ROI_CALIBRATION_STATE_INIT = 0,
        ROI_CALIBRATION_STATE_MOVE_UP = 1,
        ROI_CALIBRATION_STATE_DONE = 2
    };
}
namespace ofxCSG
{
	
	//STATIC VARS
	static float EPSILON = 1e-5;
	static float ONE_PLUS_EPSILON = EPSILON + 1;
	static float NEG_EPSILON = -EPSILON;
	
	enum Classification
	{
		UNDEFINED = 0,
		SPANNING = 1,
		FRONT = 2,
		BACK = 3,
		COPLANAR = 4
	};

	//STATIC METHODS
	template<class T>
	static T lerp(T a, T b, float k)
	{
		return a + (b - a) * k;
	}
	
	template<class T>
	static void appendVectors( vector<T>& a, vector<T>& b )
	{
		//a.reserve( a.size() + b.size() );
		a.insert( a.end(), b.begin(), b.end() );
	}
	
	static ofVec3f normalFromPoints(ofVec3f p0, ofVec3f p1, ofVec3f p2)
	{
		return (p2 - p1).cross( p0 - p1).normalize();
	}
	
	static float areaOfTriangle(ofVec3f p0, ofVec3f p1, ofVec3f p2)
	{
		return (p2 - p1).cross(p0 - p1).length() * .5;
	}
	
	static float areaOfTriangleSquared(ofVec3f p0, ofVec3f p1, ofVec3f p2)
	{
		return (p2 - p1).cross(p0 - p1).lengthSquared() * .5;
	}

	static float signedDistanceToPlane(ofVec3f point, ofVec3f planePos, ofVec3f planeNormal)
	{
		float dist = planeNormal.dot(point - planePos);
		return dist;
	}
	
	//http://geomalgorithms.com/a04-_planes.html
	static float distanceToPlane(ofVec3f point, ofVec3f planePos, ofVec3f planeNormal)
	{
		float sb, sn, sd;
		
		sn = -( planeNormal.dot(point - planePos) );
		sd = planeNormal.dot(planeNormal);
		sb = sn / sd;
		
		ofVec3f B = point + sb * planeNormal;
		
		return point.distance(B);
	}
	
	static float distanceToPlaneSigned(ofVec3f point, ofVec3f planePos, ofVec3f planeNormal)
	{
		//assumes planeNormal is a unit vector
		return -( planeNormal.dot( point - planePos ) );
		//	return -( doubleDot( planeNormal, point - planePos ) );
	}
	
	static Classification classifyPointWithPlane( ofVec3f point, ofVec3f planeNormal, float w )
	{
		float t = planeNormal.dot( point ) - w;
		return ( t < NEG_EPSILON ) ? BACK : (t > EPSILON) ? FRONT : SPANNING;
	}
	
	static Classification classifyPointWithPlane( ofVec3f point, ofVec3f planePos, ofVec3f planeNormal)
	{
		auto d = distanceToPlaneSigned( point, planePos, planeNormal );
		
		if( d > EPSILON )	return BACK;
		else if( d < NEG_EPSILON )	return FRONT;
		
		return SPANNING;
	}
	
	//barycentric coords
	//http://www.blackpawn.com/texts/pointinpoly/
	static bool getBaryCentricCoords(ofVec3f p, ofVec3f t0, ofVec3f t1, ofVec3f t2, float &u, float &v, float& w)
	{
		// Compute vectors
		ofVec3f v0 = t2 - t0;
		ofVec3f v1 = t1 - t0;
		ofVec3f v2 = p - t0;
		
		// Compute dot products
		float dot00 = v0.dot( v0 );
		float dot01 = v0.dot( v1 );
		float dot02 = v0.dot( v2 );
		float dot11 = v1.dot( v1 );
		float dot12 = v1.dot( v2 );
		
		float denom = (dot00 * dot11 - dot01 * dot01);
		
		if ( denom == 0 )
		{
			//TODO: what's the right thing to do here?
			u = v = w = 0;
			return false;
		}
		
		// Compute barycentric coordinates
		float invDenom = 1.f / denom;
		u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		v = (dot00 * dot12 - dot01 * dot02) * invDenom;
		w = 1. - u - v;
		
		return true;
	}
	
	static bool getBaryCentricCoords(ofVec3f p, ofVec3f t0, ofVec3f t1, ofVec3f t2, float &u, float &v)
	{
		float w;
		return getBaryCentricCoords(p, t0, t1, t2, u, v, w);
	}
    
    static ofVec4f getPlaneEquation(ofVec3f basePlanePos, ofVec3f basePlaneNormal){
        ofVec4f basePlaneEq;
        for(int i=0;i<3;++i)
            basePlaneEq[i]=basePlaneNormal[i];
        basePlaneEq[3]=-basePlaneNormal.dot(basePlanePos);
        return basePlaneEq;
    }
	
	static ofVec3f closestPointOnLineSegment(ofVec3f p, ofVec3f l0, ofVec3f l1)
	{
		ofVec3f diff = p - l0;
		ofVec3f dir = l1 - l0;
		float u = diff.dot( dir ) / dir.dot( dir );
		
		if ( u < 0. )	return l0;
		else if( u > 1. )	return l1;
		
		return l0 + dir * u;
	}
	
	
	//http://paulbourke.net/geometry/pointlineplane/lineline.c
	static bool LineLineIntersect( ofVec3f p1,ofVec3f p2,ofVec3f p3,ofVec3f p4, ofVec3f *pa = NULL, ofVec3f *pb = NULL )
	{
		ofVec3f p13, p43, p21;
		double d1343,d4321,d1321,d4343,d2121;
		double numer,denom;
		
		p13 = p1 - p3;
		p43 = p4 - p3;
		
		if (abs(p43.x) < EPSILON && abs(p43.y) < EPSILON && abs(p43.z) < EPSILON)	return false;
		
		p21 = p2 - p1;
		
		if (abs(p21.x) < EPSILON && abs(p21.y) < EPSILON && abs(p21.z) < EPSILON)	return false;
		
		d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
		d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
		d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
		d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
		d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;
		
		denom = d2121 * d4343 - d4321 * d4321;
		if (abs(denom) < EPSILON)	   return false;
		
		numer = d1343 * d4321 - d1321 * d4343;
		
		double mua = numer / denom;
		double mub = (d1343 + d4321 * mua) / d4343;
		
		if( pa != NULL)
		{
			*pa = p1 + mua * p21;
		}
		if( pb != NULL )
		{
			*pb = p3 + mub * p43;
		}
		
		return true;
	}
	
	static float getLineSegmentUValue(ofVec3f l0, ofVec3f l1, ofVec3f p)
	{
		ofVec3f diff = p - l0;
		ofVec3f dir = l1 - l0;
		
		if(l0 == l1)
		{
			return 0;
		}
		
		return diff.dot( dir ) / dir.dot( dir );
	}
	
	static bool isPointInLineSegment(ofVec3f l0, ofVec3f l1, ofVec3f p)
	{
		float u = getLineSegmentUValue( l0, l1, p );
		return  u >= NEG_EPSILON && u <= ONE_PLUS_EPSILON;
	}
	
	static bool intersectLineSegments(ofVec3f a0, ofVec3f a1, ofVec3f b0, ofVec3f b1, ofVec3f* intersection=NULL)
	{
		ofVec3f p;
		
		LineLineIntersect(a0, a1, b0, b1, &p);
		
		if( isPointInLineSegment(a0, a1, p) )
		{
			*intersection = p;
			return true;
		}
		
		return false;
	}
	
	static bool splitLineSegmentWithPlane( ofVec3f l0, ofVec3f l1, ofVec3f planeNormal, float w, ofVec3f* intersection)
	{
		auto c0 = classifyPointWithPlane( l0, planeNormal, w);
		auto c1 = classifyPointWithPlane( l1, planeNormal, w);
		
		if( c0 != c1 )
		{
			float k = (w - planeNormal.dot(l0)) / planeNormal.dot( l1 - l0 );
			
			*intersection = lerp( l0, l1, CLAMP(k, 0, 1) ); // the clamp fixed some errors where k > 1
			
			return true;
		}
		
		return false;
	}
	
	static int intersectLineSegmentPlane(ofVec3f p0, ofVec3f p1, ofVec3f planePos, ofVec3f planeNormal, ofVec3f* intersection = NULL)
	{
		auto d0 = distanceToPlaneSigned( p0, planePos, planeNormal );
		auto d1 = distanceToPlaneSigned( p1, planePos, planeNormal );
		
		if( (d0 >= EPSILON && d1 >= EPSILON) || ( d0 <= NEG_EPSILON && d1 <= NEG_EPSILON ) )
//		if( (d0 > 0 && d1 > 0) || ( d0 < 0 && d1 < 0 ) )
		{
			//no intersection
			return 0;
		}
		if( d0 == 0 && d1 == 0 )
		{
			//it's coplanar
			if( intersection != NULL )
			{
				*intersection = p0;
			}
			return 2;
		}
		
		//it's a hit
		if( intersection != NULL )
		{
			//lerp using the distance to plane values
			*intersection = lerp( p0, p1, d0 / (d0 - d1) );
		}
		return 1;
	}
	
	
	static bool isPointInTriangle(ofVec3f p, ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f normal )
	{
		if( fabs( distanceToPlaneSigned( p, a, normal ) ) > EPSILON )	return false;
		
		float u, v, w, epsilon = NEG_EPSILON; // 0; // EPSILON; //
		
		if( getBaryCentricCoords( p, a, b, c, u, v, w ) )
		{
			return u > epsilon && v > epsilon && w > epsilon;
		}
		
		return false;
	}
	
	static bool isPointOnPlane( ofVec3f p, ofVec3f planeNormal, float w, float epsilon = EPSILON)
	{
		float t = planeNormal.dot(p) - w;
		return abs(t) > epsilon;
	}
	
	static bool isPointInTriangle(ofVec3f p, ofVec3f a, ofVec3f b, ofVec3f c, ofVec3f normal, float epsilon )
	{
		float u, v, w;
		
		if( getBaryCentricCoords( p, a, b, c, u, v, w ) )
		{
			return u > epsilon && v > epsilon && w > epsilon;
		}
		
		return false;
	}
	
	static bool isPointInTriangle(ofVec3f p, ofVec3f a, ofVec3f b, ofVec3f c)
	{
		return isPointInTriangle( p, a, b, c, normalFromPoints(a, b, c) );
	}
	
	
	//derived from Akira-Hayasaka's ofxRayTriangleIntersection
	//	https://github.com/Akira-Hayasaka/ofxRayTriangleIntersection/blob/master/src/ofxRayTriangleIntersection.h
	//	assume ray direction is normalized
	static bool intersectRayTriangle(ofVec3f rayOrigin, ofVec3f rayDir, ofVec3f t0, ofVec3f t1, ofVec3f t2, ofVec3f* intersection=NULL)
	{
		ofVec3f normal = (t2 - t1).cross( t0 - t1).normalize();
		float vn = rayDir.dot(normal);
		
		ofVec3f diff = rayOrigin - t0;
		float xpn = diff.dot(normal);
		float distance = -xpn / vn;
		
		if (distance < 0) return false; // behind ray origin. fail
		
		ofVec3f hitPos = rayDir * distance + rayOrigin;
		
		if(isPointInTriangle(hitPos, t0, t1, t2))
		{
			//it's a hit
			if(intersection!= NULL)
			{
				*intersection = hitPos;
			}
			return true;
		}
		
		//nada
		return false;
	}
}