/*
 * opencog/embodiment/AGISimSim/SimClient.cc
 *
 * Copyright (C) 2002-2007 Novamente LLC
 * All Rights Reserved
 *
 * Written by TO_COMPLETE
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License v3 as
 * published by the Free Software Foundation and including the exceptions
 * at http://opencog.org/wiki/Licenses
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program; if not, write to:
 * Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include "util/exceptions.h"
#include "SimProxy.h"
#include <stdio.h>

#ifndef WIN32
#include <sys/time.h>
#endif

#define SEC_DELAY 0
#define USEC_DELAY 500000

#define COMMAND_BUFFER_SIZE 500
#define SCRIPT_BUFFER_SIZE 100000

bool echoing;

static timeval referenceTime;
static bool referenceTimeInitialized = false;


void initReferenceTime() {
    gettimeofday(&referenceTime, NULL);
    referenceTimeInitialized = true;
}

ulong getElapsedMillis() {
    opencog::cassert(TRACE_INFO, referenceTimeInitialized,
            "SimClient - refenceTimeInitialized should have been initialized.");
    timeval currentTime;
    gettimeofday(&currentTime, NULL);
    return (currentTime.tv_sec-referenceTime.tv_sec)*1000 + (currentTime.tv_usec-referenceTime.tv_usec)/1000;
}

unsigned int random_pos_int(int roof)
{
	return rand() % roof;
}

std::string send_random_action(SimProxy *proxy) {
    float value = (random_pos_int(5) / 10.0f);
    int a = random_pos_int(8);
    std::string result;
    switch(a) {
        case 0:
            result = proxy->strafeLeft(value);
            break;
        case 1:
            result = proxy->strafeRight(value);
            break;
        case 2:
            result = proxy->moveForward(value);
            break;
        case 3:
            result = proxy->moveBackward(value);
            break;
        case 4:
            result = proxy->turnLeft(value);
            break;
        case 5:
            result = proxy->turnRight(value);
            break;
        case 6:
            result = proxy->eyeUp(value/10);
            break;
        case 7:
            result = proxy->eyeDown(value/10);
            break;
    }
    return result;
}

bool jumping = false;
bool backjumping = false;
char* label;
std::vector<char*> backLabels;
#define GOTO_STEP 1.0
#define GOTO_WAIT 25
#define ROTATE_STEP 0.1
#define ROTATE_WAIT 25
#define TOKEN_SEPARATOR_CHARACTERS " ,()\n"

#define DEFAULT_MOVE_DISTANCE 1.0f
#define DEFAULT_TURN_ANGLE 0.7854f

#define AGENT_RADIUS 0.25f

void executeLine(SimProxy* proxy, const char* line) {
    if (jumping || backjumping) {
        if (!strcmp(line, label)) {
            //if (jumping) free(label);
            jumping = false;
            backjumping = false;
        } else {
            if (echoing) printf("Jumping line: %s\n", line);
        }
    }
    if (!strcmp(line, "random\n")) {
        if (echoing) printf("Random command:\n");
        send_random_action(proxy);
    } else if (!strcmp(line, "new\n")) {
        if (echoing) printf("New agent command:\n");
        proxy->newAgent(-300,5,-50,"pet","ball", AGENT_RADIUS);
    } else if (!strcmp(line, "reset\n")) {
        if (echoing) printf("Reset command:\n");
        proxy->resetCurrentWorld();
    } else if (!strcmp(line, "notick\n")) {
        if (echoing) printf("NoTick command:\n");
        proxy->disableAgiSimClock();
    } else if (!strcmp(line, "sense\n")) {
        if (echoing) printf("Sense command:\n");
        proxy->getCurrentSense();
    } else if (!strcmp(line, "l\n")) {
        if (echoing) printf("Turn Left command:\n");
        proxy->turnLeft(DEFAULT_TURN_ANGLE);
    } else if (!strcmp(line, "r\n")) {
        if (echoing) printf("Turn right command:\n");
        proxy->turnRight(DEFAULT_TURN_ANGLE);
    } else if (!strcmp(line, "f\n")) {
        if (echoing) printf("Move forward command:\n");
        proxy->moveForward(DEFAULT_MOVE_DISTANCE);
    } else if (!strcmp(line, "b\n")) {
        if (echoing) printf("Move backward command:\n");
        proxy->moveBackward(DEFAULT_MOVE_DISTANCE);
    } else if (!strcmp(line, "sl\n")) {
        if (echoing) printf("Strafe left command:\n");
        proxy->strafeLeft(DEFAULT_MOVE_DISTANCE);
    } else if (!strcmp(line, "sr\n")) {
        if (echoing) printf("Strafe right command:\n");
        proxy->strafeRight(DEFAULT_MOVE_DISTANCE);
    } else if (!strcmp(line, "eu\n")) {
        if (echoing) printf("Eye up command:\n");
        proxy->eyeUp(0.1);
    } else if (!strcmp(line, "ed\n")) {
        if (echoing) printf("Eye down command:\n");
        proxy->eyeDown(0.1);
    } else if (!strcmp(line, "el\n")) {
        if (echoing) printf("Eye left command:\n");
        proxy->eyeLeft(0.1);
    } else if (!strcmp(line, "er\n")) {
        if (echoing) printf("Eye Right command:\n");
        proxy->eyeRight(0.1);
    } else if (!strcmp(line, "eat\n")) {
        if (echoing) printf("Eat command:\n");
        proxy->eat("food");
    } else if (!strcmp(line, "lift\n")) {
        if (echoing) printf("Lift command:\n");
        proxy->lift("Ball");
    } else if (!strcmp(line, "drop\n")) {
        if (echoing) printf("Drop command:\n");
        proxy->drop();
    } else if (!strcmp(line, "kl\n")) {
        if (echoing) printf("Kick low command:\n");
        proxy->kickLow();
    } else if (!strcmp(line, "kh\n")) {
        if (echoing) printf("Kick high command:\n");
        proxy->kickHigh();
    } else if (!strcmp(line, "smile\n")) {
        if (echoing) printf("Smile command:\n");
        proxy->smile();
    } else if (!strcmp(line, "frown\n")) {
        if (echoing) printf("Frown command:\n");
        proxy->frown();
    } else if (!strcmp(line, "n\n")) {
        if (echoing) printf("Noise make command:\n");
        proxy->noiseMake();
    } else if (!strcmp(line, "m\n")) {
        if (echoing) printf("Message command:\n");
        proxy->message("Hello_AgiSim!");
    } else if (!strncmp(line, "wait", 4)) {
        if (echoing) printf("Wait command:\n");
        unsigned long time = atol(line+5);
        if (echoing) printf("%ld micro seconds\n", time);
        usleep(time*1000);
    } else if (!strncmp(line, "quit", 4)) {
        exit(-1);
    } else if (!strncmp(line, "backjump", 8)) {
        label = strdup(line+9);
        for (unsigned int i = 0; i < backLabels.size(); i++) {
           if (!strcmp(label, backLabels[i])) {
               if (echoing) printf("Label already used before. Ignoring...\n");
               free(label);
           }
        }
        backLabels.push_back(label);
        if (echoing) printf("Back jumping to label %s", label);
        backjumping = true;
    } else if (!strncmp(line, "jump", 4)) {
        label = strdup(line+5);
        for (unsigned int i = 0; i < backLabels.size(); i++) {
           if (!strcmp(label, backLabels[i])) {
               if (echoing) printf("Label already used before. Ignoring...\n");
               free(label);
           }
        }
        backLabels.push_back(label);
        if (echoing) printf("Jumping to label %s", label);
        jumping = true;
    } else if (!strcmp(line, "c\n")) {
        if (echoing) printf("Connecting to the server ...\n");
        std::string result;
        if (proxy->connect()) {
            result += "ok";
        } else {
            result += "error";
        }
        printf("Connection %s\n", result.c_str());
    } else if (!strncmp(line, "gotox", 5)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float initialX = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialX = %f\n", initialX);
        float y = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("y = %f\n", y);
        float z = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("z = %f\n", z);
        float finalX = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalX = %f\n", finalX);
        char msgBuffer[500];
        if (initialX != finalX) {
            float step = (finalX > initialX)?GOTO_STEP:-GOTO_STEP;
            float x = initialX;
            while (true) {
               if (step>0) {
                   if (x >= finalX) break;
               } else {
                   if (x <= finalX) break;
               }
               sprintf(msgBuffer, "setpos %s %f %f %f\n", objName, x, y, z);
               proxy->send(msgBuffer, false, false);
               usleep(GOTO_WAIT*1000);
               x+=step;
            }
            sprintf(msgBuffer, "setpos %s %f %f %f\n", objName, finalX, y, z);
            proxy->send(msgBuffer, false, false);
            usleep(GOTO_WAIT*1000);
        }
    } else if (!strncmp(line, "gotoz", 5)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float x = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("x = %f\n", x);
        float y = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("y = %f\n", y);
        float initialZ = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialZ = %f\n", initialZ);
        float finalZ = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalZ = %f\n", finalZ);
        char msgBuffer[500];
        if (initialZ != finalZ) {
            float step = (finalZ > initialZ)?GOTO_STEP:-GOTO_STEP;
            float z = initialZ;
            while(true) {
              if (step>0) {
                   if (z >= finalZ) break;
               } else {
                   if (z <= finalZ) break;
               }
               sprintf(msgBuffer, "setpos %s %f %f %f\n", objName, x, y, z);
               proxy->send(msgBuffer, false, false);
               usleep(GOTO_WAIT*1000);
               z+=step;
            }
            sprintf(msgBuffer, "setpos %s %f %f %f\n", objName, x, y, finalZ);
            proxy->send(msgBuffer, false, false);
            usleep(GOTO_WAIT*1000);
        }
    } else if (!strncmp(line, "rotatex", 7)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float initialX = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialX = %f\n", initialX);
        float y = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("y = %f\n", y);
        float z = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("z = %f\n", z);
        float finalX = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalX = %f\n", finalX);
        char msgBuffer[500];
        if (initialX != finalX) {
            float step = (finalX > initialX)?ROTATE_STEP:-ROTATE_STEP;
            float x = initialX;
            while (true) {
               if (step>0) {
                   if (x >= finalX) break;
               } else {
                   if (x <= finalX) break;
               }
               sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, x, y, z);
               proxy->send(msgBuffer, false, false);
               usleep(ROTATE_WAIT*1000);
               x+=step;
            }
            sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, finalX, y, z);
            proxy->send(msgBuffer, false, false);
            usleep(ROTATE_WAIT*1000);
        }
    } else if (!strncmp(line, "rotatey", 7)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float x = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("x = %f\n", x);
        float initialY = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialY = %f\n", initialY);
        float z = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("z = %f\n", z);
        float finalY = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalY = %f\n", finalY);
        char msgBuffer[500];
        if (initialY != finalY) {
            float step = (finalY > initialY)?ROTATE_STEP:-ROTATE_STEP;
            float y = initialY;
            while(true) {
              if (step>0) {
                   if (y >= finalY) break;
               } else {
                   if (y <= finalY) break;
               }
               sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, x, y, z);
               proxy->send(msgBuffer, false, false);
               usleep(ROTATE_WAIT*1000);
               y+=step;
            }
            sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, x, finalY, z);
            proxy->send(msgBuffer, false, false);
            usleep(ROTATE_WAIT*1000);
        }
    } else if (!strncmp(line, "rotatez", 7)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float x = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("x = %f\n", x);
        float y = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("y = %f\n", y);
        float initialZ = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialZ = %f\n", initialZ);
        float finalZ = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalZ = %f\n", finalZ);
        char msgBuffer[500];
        if (initialZ != finalZ) {
            float step = (finalZ > initialZ)?ROTATE_STEP:-ROTATE_STEP;
            float z = initialZ;
            while(true) {
              if (step>0) {
                   if (z >= finalZ) break;
               } else {
                   if (z <= finalZ) break;
               }
               sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, x, y, z);
               proxy->send(msgBuffer, false, false);
               usleep(ROTATE_WAIT*1000);
               z+=step;
            }
            sprintf(msgBuffer, "rotate %s %f %f %f\n", objName, x, y, finalZ);
            proxy->send(msgBuffer, false, false);
            usleep(ROTATE_WAIT*1000);
        }
    } else if (!strncmp(line, "move_arm", 8)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float initialAngle = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialAngle = %f\n", initialAngle);
        float finalAngle = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalAngle = %f\n", finalAngle);
        char msgBuffer[500];
        if (initialAngle != finalAngle) {
            float step = (finalAngle > initialAngle)?ROTATE_STEP:-ROTATE_STEP;
            float angle = initialAngle;
            while(true) {
              if (step>0) {
                   if (angle >= finalAngle) break;
               } else {
                   if (angle <= finalAngle) break;
               }
               sprintf(msgBuffer, "move.arm %s %f\n", objName, angle);
               proxy->send(msgBuffer, false, false);
               usleep(ROTATE_WAIT*1000);
               angle+=step;
            }
            sprintf(msgBuffer, "move.arm %s %f\n", objName, finalAngle);
            proxy->send(msgBuffer, false, false);
            usleep(ROTATE_WAIT*1000);
        }
    } else if (!strncmp(line, "move_leg", 8)) {
        char* buf;
        __strtok_r(const_cast<char*>(line),TOKEN_SEPARATOR_CHARACTERS, &buf);
        char* objName = __strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf);
        //printf("objName = %s\n", objName);
        float initialAngle = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("initialAngle = %f\n", initialAngle);
        float finalAngle = atof(__strtok_r(NULL,TOKEN_SEPARATOR_CHARACTERS, &buf));
        //printf("finalAngle = %f\n", finalAngle);
        char msgBuffer[500];
        if (initialAngle != finalAngle) {
            float step = (finalAngle > initialAngle)?ROTATE_STEP:-ROTATE_STEP;
            float angle = initialAngle;
            while(true) {
              if (step>0) {
                   if (angle >= finalAngle) break;
               } else {
                   if (angle <= finalAngle) break;
               }
               sprintf(msgBuffer, "move.leg %s %f\n", objName, angle);
               proxy->send(msgBuffer, false, false);
               usleep(ROTATE_WAIT*1000);
               angle+=step;
            }
            sprintf(msgBuffer, "move.leg %s %f\n", objName, finalAngle);
            proxy->send(msgBuffer, false, false);
            usleep(ROTATE_WAIT*1000);
        }
    } else {
        // full command
        proxy->send(line, false, true);
    }
}

void interact(SimProxy* proxy)
{
    while (proxy->IsConnected()) {
        proxy->checkAsynchronousMessages();
        char line[COMMAND_BUFFER_SIZE+1];
        printf("\nClient>> ");
        fgets (line, COMMAND_BUFFER_SIZE, stdin);
        executeLine(proxy, line);
    }
}


// The line with "%N" determines the N times that the loop will be traversed.

void test(char* script, SimProxy *proxy, int count) {
    timeval begin;
    gettimeofday(&begin, NULL);

    // Repeat script commands count times (count can be changed if it is in the script file).
    for (int i = 0; i < count; i++) {
        timeval begin;
        gettimeofday(&begin, NULL);

        char *ptr = script;
        while (*ptr != 0) {
            timeval theTime;
            gettimeofday(&theTime, NULL);
            if (echoing)
            printf("Line #%d   @%ld.%ld  ms\n", i, theTime.tv_sec*1000 + theTime.tv_usec/1000, theTime.tv_usec%1000);

            char* mountedLine;
            char line[COMMAND_BUFFER_SIZE];
            int p;
            if(backjumping) {
                line[COMMAND_BUFFER_SIZE-1] = 0;
                line[COMMAND_BUFFER_SIZE-2] = '\n';
                for (p = 2; *(ptr-p) != '\n' && (ptr-p) != script; p++) {
                    line[COMMAND_BUFFER_SIZE-p-1] = *(ptr-p);
                }
                if ((ptr-p) == script) {
                   printf("WARNING: Back jumping reached the beggining of the script\n");
                   break;
                }
                ptr -= (p-1);
                mountedLine = strdup(line+COMMAND_BUFFER_SIZE-p);
            } else {
                for (p = 0; ptr[p] != '\n' && ptr[p] != 0; p++) {
                    line[p] = ptr[p];
                }
                line[p] = '\n';
                line[p+1] = '\0';
                if (ptr[p] == '\n') {
                    ptr++;
                }
                ptr += p;
                mountedLine = strdup(line);
            }
//printf("MOUNTED LINE = '%s'\n", mountedLine);
            if (!strcmp(mountedLine, "interact\n")) {
                free(mountedLine);
                interact(proxy);
                return;
            } else if (mountedLine[0] == '%' && strlen(mountedLine) > 2) {
                count = atoi(&mountedLine[1]);
                if (echoing) printf("Looping %d times.\n", count);
                free(mountedLine);
                continue;
            } else if (mountedLine[0] == '#') {
                //Comment line.
                free(mountedLine);
                continue;
            } else if (strlen(mountedLine) > 1) {
                executeLine(proxy, mountedLine);
                free(mountedLine);
            } else {
                puts("Empty line");
                free(mountedLine);
                continue;
            }
            gettimeofday(&theTime, NULL);
            if (echoing) printf("Line ok @%ld.%ld ms\n",theTime.tv_sec*1000 + theTime.tv_usec/1000, theTime.tv_usec%1000);
        } // while (*ptr != 0)
        if (echoing) {
            timeval end;
            gettimeofday(&end, NULL);
            printf("This script round has used @%ld ms\n", (end.tv_sec-begin.tv_sec)*1000 + (end.tv_usec-begin.tv_usec)/1000);
        }
    } // for (count)
    timeval end;
    gettimeofday(&end, NULL);
    printf("Script has used @%ld ms\n", (end.tv_sec-begin.tv_sec)*1000 + (end.tv_usec-begin.tv_usec)/1000);
}

// MAIN METHOD

int main(int argc,char *argv[])
{
    srand(12345678);
    initReferenceTime();

    std::string scriptFileName = "script";
    bool remote = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "echo")) echoing = true;
        else if (!strcmp(argv[i], "remote")) remote = true;
	else scriptFileName = argv[i];
    }
    char script[SCRIPT_BUFFER_SIZE];
    FILE* f = fopen(scriptFileName.c_str(), "rt");
    if (f == NULL) {
        printf("Could not open the script file %s\n", scriptFileName.c_str());
        script[0] = 0;
    } else {
        int numRead = fread(script, sizeof(char), SCRIPT_BUFFER_SIZE-1, f);
        script[numRead] = 0;
        fclose(f);
    }

    // use the SimProxy to send messages
    SimProxy* proxy;
    if (remote) {
       proxy = new SimProxy("10.1.0.8",40002,new DefaultPerceptionAndStatusHandler(),echoing);
    } else {
       proxy = new SimProxy("localhost",40002,new DefaultPerceptionAndStatusHandler(),echoing);
    }
    if (proxy && proxy->connect()) {
        test(script,proxy,1);
    };
}



