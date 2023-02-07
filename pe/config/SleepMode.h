//=================================================================================================
/*!
 *  \file pe/config/SleepMode.h
 *  \brief Configuration of the sleeping mode
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
/*!\brief Sleep mode threshold value.
 * \ingroup config
 *
 * This value specifies the threshold value for the sleep mode of a rigid body. In case the
 * motion of a rigid body drops below this threshold, the body is put to sleep and is not moved
 * during a simulation time step. The body is wakened again if its position or its orientation is
 * changed or if forces or impulses are added to the body.\n
 * The sleep threshold can be set to any non-negative value \f$ [0..\infty) \f$. A value of zero
 * switches off sleep mode entirely.\n
 * For more details about the calculation of the motion of a rigid body, see the description of
 * the RigidBody::calcMotion() function.
 */
const real sleepThreshold = static_cast<real>( 0 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Recency-weighted average bias value for the sleep mode \f$ [0..1] \f$.
 * \ingroup config
 *
 * The motion of a rigid body is calculated by a recency-weighted average. This value specifies
 * the bias value for the calculation. It controls how much significance is given to previous
 * values. The bias value has to be in the range \f$ [0..1] \f$: A value of zero makes the
 * recency-weighted average equal to the current motion of the rigid body, a bias of one
 * ignores the current motion altogether.
 */
const real sleepBias = static_cast<real>( 0.5 );
//*************************************************************************************************
