#ifndef APP_ROLE_H_
#define APP_ROLE_H_

/* Build-time application selector - the ONE place a build picks its role.
 *
 *   APPLICATION=TAG   this module is a tag (host-attached, IMU sleep/wake,
 *                     EUI / location reporting)
 *   APPLICATION=NODE  this module is an anchor / node (PDoA ranging,
 *                     extended TWR reports)
 *
 * Set it in the project preprocessor definitions
 * (twr_demo_nRF52840.emProject -> c_preprocessor_definitions). Nothing else
 * needs to change; there must be no per-file override of it.
 *
 * Lives in its own header so headers that must not pull in app.h (the board
 * header, for example) can key off the role too.
 */
#ifndef NODE
#define NODE                1
#endif
#ifndef TAG
#define TAG                 2
#endif

#ifndef APPLICATION
#define APPLICATION         NODE
#endif

#if (APPLICATION != NODE) && (APPLICATION != TAG)
#error "APPLICATION must be set to NODE or TAG"
#endif

#endif /* APP_ROLE_H_ */
