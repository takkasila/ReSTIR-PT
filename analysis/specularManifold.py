#include "Utils/Math/MathConstants.slangh"
import numpy as np


# x1 closer to light source
# x3 closer to eye
def computeSpecularConstraintDerivatives( x1, x2, x3, dndu, dndv):

	x1T = np.array([1, 0, 0]);
	x1B = np.array([0, 1, 0]);

	x2T = np.array([0, 1, 0]);
	x2B = np.array([0, 0, 1]);

	x3T = np.array([1, 0, 0]);
	x3B = np.array([0, 0, 1]);

	x2N = np.array([1, 0, 0]);

	# Compute relevant directions and a few useful projections
	wi = x1 - x2;
	wo = x3 - x2;
	lwi = np.linalg.norm(wi);
	ili = 1 / lwi;
	ilo = 1 / np.linalg.norm(wo);
	wi *= ili;
	wo *= ilo;

	eta = 1/1.5
	# eta = 1

	# Half vector
	H = wi + eta * wo;
	ilh = 1 / np.linalg.norm(H);
	H *= ilh;

	dot_H_n = np.dot(x2N, H);
	dot_H_dndu = np.dot(dndu, H);
	dot_H_dndv = np.dot(dndv, H);
	dot_u_n = np.dot(x2T, x2N);
	dot_v_n = np.dot(x2B, x2N);

	# Local shading tangent frame
	s = x2T - dot_u_n * x2N;
	t = x2B - dot_v_n * x2N;

	ilo *= eta * ilh;
	ili *= ilh;

	# Derivatives of C with respect to x1
	dH_du = (x1T - wi * np.dot(wi, x1T)) * ili;
	dH_dv = (x1B - wi * np.dot(wi, x1B)) * ili;

	dH_du -= H * np.dot(dH_du, H);
	dH_dv -= H * np.dot(dH_dv, H);

	B1 = np.array([
		[np.dot(dH_du, s), np.dot(dH_dv, s)],
		[np.dot(dH_du, t), np.dot(dH_dv, t)]
	]);

	# Derivatives of C with respect to x2
	dH_du = -x2T * (ili + ilo)  \
			+ wi * (np.dot(wi, x2T) * ili)  \
			+ wo * (np.dot(wo, x2T) * ilo);
	dH_dv = -x2B * (ili + ilo)  \
			+ wi * (np.dot(wi, x2B) * ili)  \
			+ wo * (np.dot(wo, x2B) * ilo);

	dH_du -= H * np.dot(dH_du, H);
	dH_dv -= H * np.dot(dH_dv, H);

	B2 = np.array([
		[np.dot(dH_du, s) - np.dot(x2T, dndu) * dot_H_n - dot_u_n * dot_H_dndu,
		np.dot(dH_dv, s) - np.dot(x2T, dndv) * dot_H_n - dot_u_n * dot_H_dndv],
		[np.dot(dH_du, t) - np.dot(x2B, dndu) * dot_H_n - dot_v_n * dot_H_dndu,
		np.dot(dH_dv, t) - np.dot(x2B, dndv) * dot_H_n - dot_v_n * dot_H_dndv]
	]);

	# Derivatives of C with respect to x3
	dH_du = (x3T - wo * np.dot(wo, x3T)) * ilo;
	dH_dv = (x3B - wo * np.dot(wo, x3B)) * ilo;

	dH_du -= H * np.dot(dH_du, H);
	dH_dv -= H * np.dot(dH_dv, H);

	B3 = np.array([
		[np.dot(dH_du, s), np.dot(dH_dv, s)],
		[np.dot(dH_du, t), np.dot(dH_dv, t)]
	]);

	gradC0 = B1;
	gradC1 = B2;
	gradC2 = B3;

	# Check validity
	isValid = np.any(np.isnan(gradC0)) or np.any(np.isnan(gradC0)) or np.any(np.isnan(gradC0));

	return gradC0, gradC1, gradC2, not isValid


# x1 closer to light source
# x3 closer to eye
def computeDSDShiftJacobian(x1, x2, x3, dndu, dndv):
	# Compute Gradient C
	gradC0, gradC1, gradC2, isSuccess = computeSpecularConstraintDerivatives(x1, x2, x3, dndu, dndv);
	if (not isSuccess):
		print("Invalid gradC", 0);
		return 0;

	# NOTE: gradC0 is respective toward x3

	# Compute G'(x2 <-> x3)
	wi = x1 - x2;
	wo = x3 - x2;
	lwi = np.linalg.norm(wi);
	ili = 1 / lwi;
	lwo = np.linalg.norm(wo);
	ilo = 1 / lwo;
	wi *= ili;
	wo *= ilo;

	x2N = np.array([1, 0, 0]);

	g_x1x2 = np.dot(x2N, wo) / (lwo * lwo);
	# Compute Jacobian as G'(x2 <-> x3) * |B2^-1 * B1|
	detB2Inv = np.linalg.det(np.linalg.inv(gradC1));
	detB1 = np.linalg.det(gradC0);

	print("g_x1x2", g_x1x2);
	print("detB2Inv", detB2Inv);
	print("detB1", detB1);

	jacobian = g_x1x2 * detB2Inv * detB1;
	return jacobian;


# x1 closer to light source
# x3 closer to eye
def computeReconnectionDSDShiftJacobian():

	srcX1 = np.array([0.0413867, 0.414894, -0.275])
	srcX2 = np.array([-0.275, 0.0796242, -0.227111])
	srcX3 = np.array([-0.199877, 0, -0.21574])
	srcX2DnDu = np.zeros(3)
	srcX2DnDv = np.zeros(3)

	dstX1 = np.array([0.0413867, 0.414894, -0.275])
	dstX2 = np.array([-0.275, 0.0792535, -0.114334])
	dstX3 = np.array([-0.200301, 0, -0.0763936])
	dstX2DnDu = np.zeros(3)
	dstX2DnDv = np.zeros(3)

	dstJacobian = computeDSDShiftJacobian(dstX1, dstX2, dstX3, dstX2DnDu, dstX2DnDv);

	srcJacobian = computeDSDShiftJacobian(srcX1, srcX2, srcX3, srcX2DnDu, srcX2DnDv);

	Jacobian = dstJacobian / srcJacobian;

	print("Jacobian DSD shift dst:", dstJacobian);
	print("Jacobian DSD shift src:", srcJacobian);
	print("Jacobian Reconnection DSD shift:", Jacobian);

	return Jacobian;


if __name__ == '__main__':
	computeReconnectionDSDShiftJacobian()