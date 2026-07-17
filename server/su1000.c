// su1000 drops to the Android "system" uid/gid (1000) and execs its arguments.
//
// The scrcpy server must run as uid 1000 to create a FLAG_SECURE mirror display:
// CAPTURE_SECURE_VIDEO_OUTPUT is a signature|role permission held only by the
// platform "android" package (uid 1000); neither root (0) nor shell (2000) has
// it. su1000 must itself be started as root (adb root), which is permitted to
// change uid. A static build runs unchanged under Android's bionic loader.
#define _GNU_SOURCE
#include <grp.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: su1000 <cmd> [args...]\n");
        return 2;
    }
    gid_t groups[] = {1000, 1004, 1007, 1015, 3003, 3009};
    if (setgroups(sizeof(groups) / sizeof(groups[0]), groups) != 0 ||
        setresgid(1000, 1000, 1000) != 0 ||
        setresuid(1000, 1000, 1000) != 0) {
        perror("su1000: drop privileges");
        return 1;
    }
    execvp(argv[1], &argv[1]);
    perror("su1000: exec");
    return 127;
}
