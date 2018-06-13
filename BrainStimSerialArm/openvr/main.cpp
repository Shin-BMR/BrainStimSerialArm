#include <stdio.h>
#include <openvr.h>

bool m_rbShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];

void init()
{
	vr::IVRSystem *m_pHMD;
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
	
	while (1)
	{

		for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
		{
			vr::VRControllerState_t state;
			if (m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
			{
				m_rbShowTrackedDevice[unDevice] = state.ulButtonPressed == 0;
			}

			if (state.rAxis->x != 0.0)
			{
				printf("%d index Data : %f \n", unDevice, state.rAxis->x);
			}
		}
	}
}

void main()
{
	init();
	printf("Hello World!");
}