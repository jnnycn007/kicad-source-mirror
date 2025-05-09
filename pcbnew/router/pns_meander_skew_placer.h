/*
 * KiRouter - a push-and-(sometimes-)shove PCB router
 *
 * Copyright (C) 2013-2015 CERN
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 * Author: Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __PNS_MEANDER_SKEW_PLACER_H
#define __PNS_MEANDER_SKEW_PLACER_H

#include "pns_meander_placer.h"
#include "pns_diff_pair.h"

namespace PNS {

class ROUTER;
class SHOVE;
class OPTIMIZER;

/**
 * MEANDER_SKEW_PLACER
 *
 * Differential pair skew adjustment algorithm.
 */
class MEANDER_SKEW_PLACER : public MEANDER_PLACER
{
public:
    MEANDER_SKEW_PLACER( ROUTER* aRouter );
    ~MEANDER_SKEW_PLACER();

    /// @copydoc PLACEMENT_ALGO::Start()
    bool Start( const VECTOR2I& aP, ITEM* aStartItem ) override;

    /// @copydoc PLACEMENT_ALGO::Move()
    bool Move( const VECTOR2I& aP, ITEM* aEndItem ) override;

    /// @copydoc MEANDER_PLACER_BASE::TuningLengthResult()
    long long int TuningLengthResult() const override;

    /// @copydoc MEANDER_PLACER_BASE::TuningDelayResult()
    int64_t TuningDelayResult() const override;

    long long int CurrentSkew() const;

private:
    long long int origPathLength() const override;

    int64_t origPathDelay() const override;

    void calculateTimeDomainTargets() override;

    DIFF_PAIR m_originPair;
    ITEM_SET  m_tunedPathP, m_tunedPathN;

    long long int m_coupledLength;
    int64_t       m_coupledDelay;
    int           m_padToDieLengthP;
    int           m_padToDieLengthN;
    int64_t       m_padToDieDelayP;
    int64_t       m_padToDieDelayN;
};

}

#endif    // __PNS_MEANDER_SKEW_PLACER_H
