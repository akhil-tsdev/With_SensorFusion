/*
 * Copyright (c) 2015 TuringSense
 * All rights reserved.
 *
 * cwati - Sep 17, 2015
 * 
 * frame_type.h
 *
 */

#ifndef __FRAME_TYPE_H__
#define __FRAME_TYPE_H__

typedef enum {
	sensorRH10 = 1,		/* 10-sensor right handed 
							right shank
							right thigh
							right back palm (left side for left-handed player)
							right wrist (left side for left-handed player)
							right upper arm (left side for left-handed player)
							right shoulder (left side for left-handed player)
							trunk
							head
							left thigh
							left shank
							*/
	sensorLF10,		/* 10-sensor left handed */
	sensorUB5,		/* 5-sensor upper body 
                     Right hand
                     Right lower arm
                     Right upper arm
                     trunk
                     Right shoulder */
	sensorLB5,		/* 5-sensor lower body */
    sensorMD1,      /* 5-sensor for motion detection
                     Right hand
                     Right lower arm
                     trunk
                     Right upper leg
                     Right lower leg  */
    sensorMD2,      /* 5-sensor for motion detection
                     Right hand
                     Right lower arm
                     Right upper arm
                     trunk
                     Right lower leg */
    sensorMD3,      /* 4 - sensor for motion detection
                     Right lower arm
                     trunk
                     Right upper leg
                     Right lower leg
                     SDK codes will copy right lower arm data to right hand
                     */
    sensorMD7,      /* 7 sensors for motion detection
                     Right Hand
                     Right lower arm
                     right upper arm
                     Chest
                     RightShoulder
                     pelvis
                     racquet */
    sensorUB7,      /*  7 sensors for upper body
                     right foreArm
                     right upperArm
                     right shoulder
                     Trunk/Chest
                     Left shouler
                     left upper arm
                     left forearm */
    sensorTUB10,    /* 10 sensor for totally upper body
                     right hand
                     right forearm
                     right upper arm
                     right shoulder
                     trunk/chest
                     left shoulder
                     left upper arm
                     left forearm
                     left hand
                     head */
    sensorUBR6,     /* 6 sensor for UB5 + racquet
                     Right hand
                     Right lower arm
                     Right upper arm
                     trunk
                     Right shoulder
                     racquet */
    sensorLUBR6,    /* 6 sensors for left UB5 + racquet
                     left Hand
                     left lower arm
                     left upper arm
                     trunk
                     left shoulder
                     racquet */
    sensorLMD7,     /* 7 sensors for motion detection
                     left Hand
                     left lower arm
                     left upper arm
                     Chest
                     left shoulder
                     pelvis
                     racquet */
    sensorFBR16,    /* 16 sensors
                     right lower leg
                     right upper leg
                     right hand
                     right lower arm
                     right upper arm
                     right shoulder
                     head
                     chest
                     pelvis
                     left shoulder
                     left upper arm
                     left lower arm
                     left hand
                     left upper leg
                     left lower leg
                     object/racket*/
                     
} frame_type_t;

#endif /* __FRAME_TYPE_H__ */
