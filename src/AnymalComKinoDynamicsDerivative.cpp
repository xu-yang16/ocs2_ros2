/*
 * AnymalComKinoDynamicsDerivative.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics/AnymalComKinoDynamicsDerivative.h"

namespace anymal {

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const std::array<bool,4>& stanceLegs,
		const double& gravitationalAcceleration, const switched_model::Options& options,
		const switched_model::FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner,
		const std::vector<switched_model::EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints)

: Base(new AnymalKinematics, new AnymalCom,
		stanceLegs, gravitationalAcceleration,
		options, feetZDirectionPlanner, endEffectorStateConstraints)
{}

AnymalComKinoDynamicsDerivative::AnymalComKinoDynamicsDerivative(const AnymalComKinoDynamicsDerivative& rhs)
: Base(rhs)
{}

std::shared_ptr<typename AnymalComKinoDynamicsDerivative::Base::Base> AnymalComKinoDynamicsDerivative::clone() const {

	return std::allocate_shared< AnymalComKinoDynamicsDerivative, Eigen::aligned_allocator<AnymalComKinoDynamicsDerivative> > (
			Eigen::aligned_allocator<AnymalComKinoDynamicsDerivative>(), *this);
}

} //end of namespace anymal
