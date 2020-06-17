// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui & Rosen Diankov (rosen.diankov@gmail.com)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "libopenrave.h"
#include <openrave/posturedescriber.h>

namespace OpenRAVE {

void RobotBase::_EnsureEssentialKinematicsChainRegisteredOnManipulator(ManipulatorConstPtr pmanip) {
    if(!_mEssentialLinkPairs.count(pmanip)) {
        _mEssentialLinkPairs[pmanip] = ExtractEssentialKinematicsChain(pmanip);
    }
}

bool RobotBase::UnregisterPostureDescriber(const LinkPair& kinematicsChain) {
    return this->SetPostureDescriber(kinematicsChain, nullptr);
}

bool RobotBase::UnregisterPostureDescriber(ManipulatorConstPtr pmanip) {
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->UnregisterPostureDescriber(_mEssentialLinkPairs.at(pmanip));
}

bool RobotBase::SetPostureDescriber(const LinkPair& kinematicsChain, PostureDescriberBasePtr pDescriber)
{
    if (pDescriber == nullptr) {
        if (_mPostureDescribers.count(kinematicsChain)) {
            _mPostureDescribers.erase(kinematicsChain); // remove instead of setting null solver
        }
        return true;
    }

    const LinkPtr& baselink = kinematicsChain[0];
    const LinkPtr& eelink = kinematicsChain[1];

    if (!pDescriber->Supports(kinematicsChain)) {
        throw OPENRAVE_EXCEPTION_FORMAT("Describer does not support kinematics chain from \"%s\" to eelink \"%s\"",
                                        baselink->GetName() % eelink->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }

    const LinkPair& kinematicsChainDescribed = pDescriber->GetEssentialKinematicsChain();
    if(kinematicsChainDescribed != kinematicsChain) {
        throw OPENRAVE_EXCEPTION_FORMAT("Kinematics chains do not match: describer has baselink \"%s\" and eelink \"%s\"; "
                                        "input has baselink \"%s\" and eelink \"%s\"",
                                        kinematicsChainDescribed[0]->GetName() % kinematicsChainDescribed[1]->GetName() %
                                        baselink->GetName() % eelink->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }

    _mPostureDescribers[kinematicsChain] = pDescriber;
    return true;
}

bool RobotBase::SetPostureDescriber(ManipulatorConstPtr pmanip, PostureDescriberBasePtr pDescriber)
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->SetPostureDescriber(_mEssentialLinkPairs.at(pmanip), pDescriber);
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(const LinkPair& kinematicsChain)
{
    if(_mPostureDescribers.count(kinematicsChain)) {
        return _mPostureDescribers.at(kinematicsChain);
    }
    const LinkPair essentialKinematicsChain = ExtractEssentialKinematicsChain(kinematicsChain);
    return _mPostureDescribers.count(essentialKinematicsChain) ? _mPostureDescribers.at(essentialKinematicsChain) : PostureDescriberBasePtr();
}

PostureDescriberBasePtr RobotBase::GetPostureDescriber(ManipulatorConstPtr pmanip)
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be nullptr", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    _EnsureEssentialKinematicsChainRegisteredOnManipulator(pmanip);
    return this->GetPostureDescriber(_mEssentialLinkPairs.at(pmanip));
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, const LinkPair& kinematicsChain, const std::vector<double>& dofvalues) const
{
    if(!_mPostureDescribers.count(kinematicsChain)) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("failed to find robot posture describer for links from \"%s\" to \"%s\" for robot \"%s\""),
                                        GetName() % kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(), OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    const PostureDescriberBasePtr& pDescriber = _mPostureDescribers.at(kinematicsChain);
    const LinkPair& kinematicsChainDescribed = pDescriber->GetEssentialKinematicsChain();
    if(kinematicsChainDescribed != kinematicsChain) {
        throw OPENRAVE_EXCEPTION_FORMAT("Kinematics chains do not match: describer has baselink \"%s\" and eelink \"%s\"; "
                                        "input has baselink \"%s\" and eelink \"%s\"",
                                        kinematicsChainDescribed[0]->GetName() % kinematicsChainDescribed[1]->GetName() %
                                        kinematicsChain[0]->GetName() % kinematicsChain[1]->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return pDescriber->ComputePostureStates(posturestates, dofvalues);
}

bool RobotBase::ComputePostureStates(std::vector<PostureStateInt>& posturestates, ManipulatorConstPtr pmanip, const std::vector<double>& dofvalues) const
{
    if(pmanip == nullptr) {
        throw OPENRAVE_EXCEPTION_FORMAT0("Input manipulator cannot be null", OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    if(!_mEssentialLinkPairs.count(pmanip)) {
        throw OPENRAVE_EXCEPTION_FORMAT("Have not included the mapping from manipulator %s to its essential kinematics chain yet",
                                        pmanip->GetName(),
                                        OpenRAVEErrorCode::ORE_InvalidArguments);
    }
    return this->ComputePostureStates(posturestates, _mEssentialLinkPairs.at(pmanip), dofvalues);
}

#ifndef M_TWO_PI
#define M_TWO_PI 6.2831853071795864769252
#endif

void RobotBase::Manipulator::SetupMultiTurnCacheValues()
{
    const RobotBasePtr probot = this->GetRobot();
    const RobotBase::RobotStateSaver saver(probot);
    const std::vector<int>& vArmIndices = this->GetArmIndices();
    probot->GetDOFLimits(_qlower, _qupper, vArmIndices);
    const size_t armdof = this->GetArmDOF();
    _vjointtypes.resize(armdof, 0);
    _qbigrangeindices.clear();
    _qbigrangemaxsols.clear();
    _qbigrangemaxcumprod = {1};
    for(size_t i = 0; i < armdof; ++i) {
        const int armindex = vArmIndices[i];
        const KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(armindex);
        const int dofindex = pjoint->GetDOFIndex();
        const int iaxis = armindex - dofindex;
        _vjointtypes[i] = pjoint->IsPrismatic(iaxis) ? 0 : pjoint->IsCircular(iaxis) ? 2 : 1

        if(_vjointtypes[i] != 1) {
            continue;
        }

        const double qrange = _qupper[i] - _qlower[i];
        if( qrange > M_TWO_PI ) {
            // ensure pjoint is revolute (1 dof), neither prismatic nor circular (3 dof)
            _qbigrangeindices.push_back(i);
            const int npossiblejvals = 1 + int(qrange/M_TWO_PI);
            _qbigrangemaxsols.push_back(npossiblejvals); // max redundant solutions
            _qbigrangemaxcumprod.push_back(_qbigrangemaxcumprod.back() * npossiblejvals);
        }
    } 
}

bool RobotBase::Manipulator::_CheckDOFValues(std::vector<double>& jvals) const
{
    const size_t armdof = this->GetArmDOF();
    if(jvals.size() != armdof) {
        return false;
    }

    for (size_t i = 0; i < armdof; ++i) {
        double& jval = jvals[i];
        if (_vjointtypes[i] == 2) {
            // circular
            while (jval > M_PI) {
                jval -= M_TWO_PI;
            }
            while (jval <= -M_PI) {
                jval += M_TWO_PI;
            }
        }
        else if (_vjointtypes[i] == 1) {
            // revolute
            while (jval > _qupper[i]) {
                jval -= M_TWO_PI;
            }
            while (jval < _qlower[i]) {
                jval += M_TWO_PI;
            }
            // in this case jval may not be in (-pi, pi]
        }
        // due to error propagation, give error bounds for lower and upper limits
        if (jval < _qlower[i] - g_fEpsilonJointLimit || jval > _qupper[i] + g_fEpsilonJointLimit) {
            return false;
        }
    }
    return true;
}

int RobotBase::Manipulator::ComputeMultiTurnIndex(const std::vector<double>& jvals)
{    
    std::vector<double> solbase = jvals;
    if(!_CheckDOFValues(solbase)) {
        return -1; // invalid
    }

    if(_qbigrangeindices.empty()) {
        return 0; // no revolute/circular joint has range >= 2*pi
    }

    const size_t nindices = _qbigrangeindices.size(); // > 0
    size_t prod = 1;
    int multiTurnIndex = 0;

    for(size_t i = 0; i < nindices; ++i) {
        const size_t index = _qbigrangeindices[i];
        const double fbase = solbase[index];
        int multiTurnSubIndex = -1;
        int count = 0;

        if(MATH_FABS(jvals[index] - fbase) <= g_fEpsilonJointLimit) {
            multiTurnSubIndex = count;
        }
        
        if(multiTurnSubIndex == -1) {
            for(double f = fbase - M_TWO_PI; f >= _qlower[index]; f -= M_TWO_PI) {
                ++count;
                if(MATH_FABS(jvals[index] - fbase) <= g_fEpsilonJointLimit) {
                    multiTurnSubIndex = count;
                    break;
                }
            }
        }

        if(multiTurnSubIndex == -1) {
            for(double f = fbase + M_TWO_PI; f <= _qupper[index]; f += M_TWO_PI) {
                ++count;
                if(MATH_FABS(jvals[index] - fbase) <= g_fEpsilonJointLimit) {
                    multiTurnSubIndex = count;
                    break;
                }
            }
        }

        OPENRAVE_ASSERT_FORMAT0(multiTurnSubIndex != -1, "Must have set multiTurnSubIndex", ORE_InvalidState);

        // to recall, _qbigrangemaxsols[k] is  1 + int((_qupper[i]-_qlower[i])/M_TWO_PI);
        OPENRAVE_ASSERT_OP_FORMAT(multiTurnSubIndex, <=, _qbigrangemaxsols[i],
                                  "exceeded max possible redundant solutions for manip arm index %d",
                                  multiTurnSubIndex, ORE_InconsistentConstraints);

        multiTurnIndex += multiTurnSubIndex * _qbigrangemaxcumprod[i];
    }

    /*
       Assume that j0, j1, ..., j5 each have {2, 1, 3, 2, 1, 4} solutions.

       Assume _qbigrangeindices = {0, 2, 3, 4, 5}, and hence nindices = 5.

       Hence {2, 3, 2, 1, 4}, and nTotal = 2x3x2x1x4 = 48.

       The simpler algorithm works as follows. Say index = 35.

       35 / 2 = 17 ... 1
       17 / 3 =  5 ... 2
       5  / 2 =  2 ... 1
       2  / 1 =  2 ... 0  (skip, as remainder is always 0)
       2  / 4 =  0 ... 2

       So we have indices [1, 2, 1, 0, 2].

       Assume vpossiblejvals = {3, 3, 2, 2, 4}. Then _qbigrangemaxcumprod = {1, 3, 9, 18, 36, 144}.

       Then p.second stores

       index = 1*1 + 2*3 + 1*9 + 0*18 + 2*36 = 88.

     */
    return multiTurnIndex;
}

} // end namespace OpenRAVE
