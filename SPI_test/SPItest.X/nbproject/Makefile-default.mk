#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile

# Environment
MKDIR=mkdir -p
RM=rm -f 
CP=cp 
# Macros
CND_CONF=default

ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof
else
IMAGE_TYPE=production
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof
endif
# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}
# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/Untitled.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

# Path to java used to run MPLAB X when this makefile was created
MP_JAVA_PATH=C:\\Program\ Files\\Java\\jre6/bin/
OS_ORIGINAL="MINGW32_NT-5.1"
OS_CURRENT="$(shell uname -s)"
############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
MP_CC=C:\\MCC18\\bin\\mcc18.exe
# MP_BC is not defined
MP_AS=C:\\MCC18\\bin\\..\\mpasm\\MPASMWIN.exe
MP_LD=C:\\MCC18\\bin\\mplink.exe
MP_AR=C:\\MCC18\\bin\\mplib.exe
# MP_BC is not defined
MP_CC_DIR=C:\\MCC18\\bin
# MP_BC_DIR is not defined
MP_AS_DIR=C:\\MCC18\\bin\\..\\mpasm
MP_LD_DIR=C:\\MCC18\\bin
MP_AR_DIR=C:\\MCC18\\bin
# MP_BC_DIR is not defined
# This makefile will use a C preprocessor to generate dependency files
MP_CPP=C:/Program\ Files/Microchip/MPLABX/mplab_ide/mplab_ide/modules/../../bin/mplab-cpp
.build-conf: ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof

MP_PROCESSOR_OPTION=18F4550
MP_PROCESSOR_OPTION_LD=18f4550
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: createRevGrep
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
__revgrep__:   nbproject/Makefile-${CND_CONF}.mk
	@echo 'grep -q $$@' > __revgrep__
	@echo 'if [ "$$?" -ne "0" ]; then' >> __revgrep__
	@echo '  exit 0' >> __revgrep__
	@echo 'else' >> __revgrep__
	@echo '  exit 1' >> __revgrep__
	@echo 'fi' >> __revgrep__
	@chmod +x __revgrep__
else
__revgrep__:   nbproject/Makefile-${CND_CONF}.mk
	@echo 'grep -q $$@' > __revgrep__
	@echo 'if [ "$$?" -ne "0" ]; then' >> __revgrep__
	@echo '  exit 0' >> __revgrep__
	@echo 'else' >> __revgrep__
	@echo '  exit 1' >> __revgrep__
	@echo 'fi' >> __revgrep__
	@chmod +x __revgrep__
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	${MKDIR} ${OBJECTDIR}/_ext/1472 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -I"../../../MCC18/h" -ou- -ot- -ob- -op- -or- -od- -opa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/main.o   ../main.c  > ${OBJECTDIR}/_ext/1472/main.err 2>&1 ; if [ $$? -eq 0 ] ; then cat ${OBJECTDIR}/_ext/1472/main.err | sed 's/\(^.*:.*:\)\(Warning\)\(.*$$\)/\1 \2:\3/g' ; else cat ${OBJECTDIR}/_ext/1472/main.err | sed 's/\(^.*:.*:\)\(Error\)\(.*$$\)/\1 \2:\3/g' ; exit 1 ; fi
	${MP_CPP}  -MMD ${OBJECTDIR}/_ext/1472/main.o.temp ../main.c __temp_cpp_output__ -D __18F4550 -D __18CXX -I C:\\MCC18\\h -I C:\\MCC18\\bin/../h  -D__18F4550
	printf "%s/" ${OBJECTDIR}/_ext/1472 > ${OBJECTDIR}/_ext/1472/main.o.d
ifneq (,$(findstring MINGW32,$(OS_CURRENT)))
	cat ${OBJECTDIR}/_ext/1472/main.o.temp | sed -e 's/\\\ /__SPACES__/g' -e's/\\$$/__EOL__/g' -e 's/\\/\//g' -e 's/__SPACES__/\\\ /g' -e 's/__EOL__/\\/g' >> ${OBJECTDIR}/_ext/1472/main.o.d
else
	cat ${OBJECTDIR}/_ext/1472/main.o.temp >> ${OBJECTDIR}/_ext/1472/main.o.d
endif
	${RM} __temp_cpp_output__
${OBJECTDIR}/_ext/1472/Untitled.o: ../Untitled.c  nbproject/Makefile-${CND_CONF}.mk
	${RM} ${OBJECTDIR}/_ext/1472/Untitled.o.d 
	${MKDIR} ${OBJECTDIR}/_ext/1472 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG  -p$(MP_PROCESSOR_OPTION) -I"../../../MCC18/h" -ou- -ot- -ob- -op- -or- -od- -opa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/Untitled.o   ../Untitled.c  > ${OBJECTDIR}/_ext/1472/Untitled.err 2>&1 ; if [ $$? -eq 0 ] ; then cat ${OBJECTDIR}/_ext/1472/Untitled.err | sed 's/\(^.*:.*:\)\(Warning\)\(.*$$\)/\1 \2:\3/g' ; else cat ${OBJECTDIR}/_ext/1472/Untitled.err | sed 's/\(^.*:.*:\)\(Error\)\(.*$$\)/\1 \2:\3/g' ; exit 1 ; fi
	${MP_CPP}  -MMD ${OBJECTDIR}/_ext/1472/Untitled.o.temp ../Untitled.c __temp_cpp_output__ -D __18F4550 -D __18CXX -I C:\\MCC18\\h -I C:\\MCC18\\bin/../h  -D__18F4550
	printf "%s/" ${OBJECTDIR}/_ext/1472 > ${OBJECTDIR}/_ext/1472/Untitled.o.d
ifneq (,$(findstring MINGW32,$(OS_CURRENT)))
	cat ${OBJECTDIR}/_ext/1472/Untitled.o.temp | sed -e 's/\\\ /__SPACES__/g' -e's/\\$$/__EOL__/g' -e 's/\\/\//g' -e 's/__SPACES__/\\\ /g' -e 's/__EOL__/\\/g' >> ${OBJECTDIR}/_ext/1472/Untitled.o.d
else
	cat ${OBJECTDIR}/_ext/1472/Untitled.o.temp >> ${OBJECTDIR}/_ext/1472/Untitled.o.d
endif
	${RM} __temp_cpp_output__
else
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	${MKDIR} ${OBJECTDIR}/_ext/1472 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"../../../MCC18/h" -ou- -ot- -ob- -op- -or- -od- -opa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/main.o   ../main.c  > ${OBJECTDIR}/_ext/1472/main.err 2>&1 ; if [ $$? -eq 0 ] ; then cat ${OBJECTDIR}/_ext/1472/main.err | sed 's/\(^.*:.*:\)\(Warning\)\(.*$$\)/\1 \2:\3/g' ; else cat ${OBJECTDIR}/_ext/1472/main.err | sed 's/\(^.*:.*:\)\(Error\)\(.*$$\)/\1 \2:\3/g' ; exit 1 ; fi
	${MP_CPP}  -MMD ${OBJECTDIR}/_ext/1472/main.o.temp ../main.c __temp_cpp_output__ -D __18F4550 -D __18CXX -I C:\\MCC18\\h -I C:\\MCC18\\bin/../h  -D__18F4550
	printf "%s/" ${OBJECTDIR}/_ext/1472 > ${OBJECTDIR}/_ext/1472/main.o.d
ifneq (,$(findstring MINGW32,$(OS_CURRENT)))
	cat ${OBJECTDIR}/_ext/1472/main.o.temp | sed -e 's/\\\ /__SPACES__/g' -e's/\\$$/__EOL__/g' -e 's/\\/\//g' -e 's/__SPACES__/\\\ /g' -e 's/__EOL__/\\/g' >> ${OBJECTDIR}/_ext/1472/main.o.d
else
	cat ${OBJECTDIR}/_ext/1472/main.o.temp >> ${OBJECTDIR}/_ext/1472/main.o.d
endif
	${RM} __temp_cpp_output__
${OBJECTDIR}/_ext/1472/Untitled.o: ../Untitled.c  nbproject/Makefile-${CND_CONF}.mk
	${RM} ${OBJECTDIR}/_ext/1472/Untitled.o.d 
	${MKDIR} ${OBJECTDIR}/_ext/1472 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -I"../../../MCC18/h" -ou- -ot- -ob- -op- -or- -od- -opa-  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/1472/Untitled.o   ../Untitled.c  > ${OBJECTDIR}/_ext/1472/Untitled.err 2>&1 ; if [ $$? -eq 0 ] ; then cat ${OBJECTDIR}/_ext/1472/Untitled.err | sed 's/\(^.*:.*:\)\(Warning\)\(.*$$\)/\1 \2:\3/g' ; else cat ${OBJECTDIR}/_ext/1472/Untitled.err | sed 's/\(^.*:.*:\)\(Error\)\(.*$$\)/\1 \2:\3/g' ; exit 1 ; fi
	${MP_CPP}  -MMD ${OBJECTDIR}/_ext/1472/Untitled.o.temp ../Untitled.c __temp_cpp_output__ -D __18F4550 -D __18CXX -I C:\\MCC18\\h -I C:\\MCC18\\bin/../h  -D__18F4550
	printf "%s/" ${OBJECTDIR}/_ext/1472 > ${OBJECTDIR}/_ext/1472/Untitled.o.d
ifneq (,$(findstring MINGW32,$(OS_CURRENT)))
	cat ${OBJECTDIR}/_ext/1472/Untitled.o.temp | sed -e 's/\\\ /__SPACES__/g' -e's/\\$$/__EOL__/g' -e 's/\\/\//g' -e 's/__SPACES__/\\\ /g' -e 's/__EOL__/\\/g' >> ${OBJECTDIR}/_ext/1472/Untitled.o.d
else
	cat ${OBJECTDIR}/_ext/1472/Untitled.o.temp >> ${OBJECTDIR}/_ext/1472/Untitled.o.d
endif
	${RM} __temp_cpp_output__
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) ../18f4550.lkr   -w -x -l"../../../MCC18/lib"  -z__MPLAB_BUILD=1  -z__MPLAB_DEBUG=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof ${OBJECTFILES}      
else
dist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) ../18f4550.lkr   -w  -l"../../../MCC18/lib"  -z__MPLAB_BUILD=1  -l ${MP_CC_DIR}\\..\\lib  -odist/${CND_CONF}/${IMAGE_TYPE}/SPItest.X.${IMAGE_TYPE}.cof ${OBJECTFILES}      
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
