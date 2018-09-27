//-*-Mode: C++;-*-
#pragma once

//SDK

#ifdef _WIN32
	#ifdef _A2KERNEL_DEF
		#define A2BEHAVIORALEXPORT __declspec(dllexport)
	#else
		#ifdef _A2BEHAVIORAL_DEF
			#define A2BEHAVIORALEXPORT __declspec(dllexport)
		#else
			#define A2BEHAVIORALEXPORT __declspec(dllimport)
		#endif
	#endif

	#pragma warning(disable: 4251)
	#pragma warning(disable: 4786)
	#pragma warning(disable: 4503)
//	#pragma warning(disable: 4284)
#else
	#define A2BEHAVIORALEXPORT
#endif
