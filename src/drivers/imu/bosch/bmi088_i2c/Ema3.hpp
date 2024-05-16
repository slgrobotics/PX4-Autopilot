/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 *  @author Sergei Grichine <slg@quakemap.com>
 */

#pragma once

#include <float.h>

template<typename Type>
class Ema3
{

public:
	inline Ema3() {};
	inline ~Ema3() {};

	inline void init(int period)
	{
		emaPeriod = period;
		multiplier = 2.0 / (1.0 + emaPeriod);
		valuePrev = {NAN, NAN, NAN};
	};

	const inline matrix::Vector3<Type> Compute(matrix::Vector3<Type> val)
	{
		matrix::Vector3<Type> valEma{};

		for (int i = 0; i < 3; i++) {
			valEma(i) = emaPeriod <= 1
				    || !PX4_ISFINITE(valuePrev(i)) ? val(i) : ((val(i) - valuePrev(i)) * multiplier + valuePrev(i));
			valuePrev(i) = valEma(i);
		}

		return valEma;
	};

	inline matrix::Vector3<Type> ValuePrev() { return valuePrev; };

	inline void Reset()	{ valuePrev = {NAN, NAN, NAN}; };

private:
	// variables :
	matrix::Vector3<Type> valuePrev{NAN, NAN, NAN};
	int emaPeriod{0};
	Type multiplier;
};

using Ema3f = Ema3<float>;
