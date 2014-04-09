// C++ lambda versions of various chipmunk functions with callbacks 
// Contributed in 2014, Jelmer Cnossen
#pragma once

#include <functional>


template<typename TContainer, typename TElem>
struct CP_BodyWrapper { // wrapper with container
	static void lambdaWrapper(TContainer* b,TElem* s, void *d) { (*(std::function<void(TElem*)>*)d)(s); } 
};
inline void cpBodyEachShape(cpBody* body, std::function<void(cpShape*)> callback) {
	cpBodyEachShape(body, CP_BodyWrapper<cpBody, cpShape>::lambdaWrapper, &callback); 
}
inline void cpBodyEachConstraint(cpBody* body, std::function<void(cpConstraint*)> callback) {
	cpBodyEachConstraint(body, CP_BodyWrapper<cpBody, cpConstraint>::lambdaWrapper, &callback); 
}
inline void cpBodyEachArbiter(cpBody* body, std::function<void(cpArbiter*)> callback) {
	cpBodyEachArbiter(body, CP_BodyWrapper<cpBody, cpArbiter>::lambdaWrapper, &callback); 
}



template<typename TElem> struct CP_SpaceWrapper { // wrapper without container
	static void lambdaWrapper(TElem* s, void *d) { (*(std::function<void(TElem*)>*)d)(s); } 
};
inline void cpSpaceEachBody(cpSpace* s, std::function<void(cpBody*)> callback) {
	cpSpaceEachBody(s, CP_SpaceWrapper<cpBody>::lambdaWrapper, &callback); 
}
inline void cpSpaceEachShape(cpSpace* s, std::function<void(cpShape*)> callback) {
	cpSpaceEachShape(s, CP_SpaceWrapper<cpShape>::lambdaWrapper, &callback); 
}
inline void cpSpaceEachConstraint(cpSpace* s, std::function<void(cpConstraint*)> callback) {
	cpSpaceEachConstraint(s, CP_SpaceWrapper<cpConstraint>::lambdaWrapper, &callback); 
}


typedef std::function<void(cpShape*shape, cpFloat distance, cpVect point)> cpSpaceNearestPointQueryLambda;
inline void cpSpaceNearestPointQuery(cpSpace *space, cpVect point, cpFloat maxDistance, cpLayers layers, cpGroup group, cpSpaceNearestPointQueryLambda callback) {
	struct wrap {
		static void f(cpShape* shape, cpFloat distance, cpVect point, void *d) { (*(cpSpaceNearestPointQueryLambda*)d)(shape, distance, point); }	
	};
	cpSpaceNearestPointQuery(space, point, maxDistance, layers, group, wrap::f, &callback);
}

typedef std::function<void(cpShape* shape, cpFloat t, cpVect n)> cpSpaceSegmentQueryLambda;
inline void cpSpaceSegmentQuery(cpSpace *space, cpVect start, cpVect end, cpLayers layers, cpGroup group, cpSpaceSegmentQueryLambda callback) {
	struct wrap {
		static void f(cpShape* shape, cpFloat t, cpVect n, void *d) { (*(cpSpaceSegmentQueryLambda*)d)(shape, t, n); }	
	};
	cpSpaceSegmentQuery(space, start, end, layers, group, wrap::f, &callback);
}

typedef std::function<void (cpShape* shape)> cpSpaceBBQueryLambda;
inline void cpSpaceBBQuery(cpSpace *space, cpBB bb, cpLayers layers, cpGroup group, cpSpaceBBQueryLambda callback) {
	struct wrap {
		static void f(cpShape* shape, void *d) { (*(cpSpaceBBQueryLambda*)d)(shape); }	
	};
	cpSpaceBBQuery(space, bb, layers, group, wrap::f, &callback);
}

typedef std::function<void(cpShape* shape, cpContactPointSet *points)> cpShapeShapeQueryLambda;
inline cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpShapeShapeQueryLambda callback) {
	struct wrap {
		static void f(cpShape* shape, cpContactPointSet* points, void *d) { (*(cpShapeShapeQueryLambda*)d)(shape, points); }	
	};
	return cpSpaceShapeQuery(space,shape, wrap::f, &callback);
}

