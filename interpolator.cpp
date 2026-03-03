#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
    // Convert angles to radians
    double rx = angles[0] * M_PI / 180.0;
    double ry = angles[1] * M_PI / 180.0;
    double rz = angles[2] * M_PI / 180.0;
    
    double sx = sin(rx), cx = cos(rx);
    double sy = sin(ry), cy = cos(ry);
    double sz = sin(rz), cz = cos(rz);
    
    // Compute the rotation matrix
    R[0] = cz * cy;
    R[1] = cz * sy * sx - sz * cx;
    R[2] = cz * sy * cx + sz * sx;
    R[3] = sz * cy;
    R[4] = sz * sy * sx + cz * cx;
    R[5] = sz * sy * cx - cz * sx;
    R[6] = -sy;
    R[7] = cy * sx;
    R[8] = cy * cx;
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();
    int startKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
   
        int prevKeyframe = startKeyframe - (N + 1);
        if (prevKeyframe < 0) prevKeyframe = startKeyframe;
        
        int nextNextKeyframe = endKeyframe + (N + 1);
        if (nextNextKeyframe >= inputLength) nextNextKeyframe = endKeyframe;
        
        Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
        Posture * pStart = pInputMotion->GetPosture(startKeyframe);
        Posture * pEnd = pInputMotion->GetPosture(endKeyframe);
        Posture * pNextNext = pInputMotion->GetPosture(nextNextKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *pStart);
        pOutputMotion->SetPosture(endKeyframe, *pEnd);
        
        // interpolate in between
        for(int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            
            // Calculate control points
            vector a_n_root = pStart->root_pos + (pEnd->root_pos -  pPrev->root_pos) * (1.0 / 6.0);
            vector b_n_root = pEnd->root_pos - (pNextNext->root_pos - pStart->root_pos) * (1.0 / 6.0);
            
            // interpolate root position
            interpolatedPosture.root_pos = DeCasteljauEuler(t, pStart->root_pos, a_n_root, b_n_root, pEnd->root_pos);
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                // Calculate control points
                vector a_n_bone = pStart->bone_rotation[bone] + (pEnd->bone_rotation[bone] - pPrev->bone_rotation[bone]) * (1.0 / 6.0);
                vector b_n_bone = pEnd->bone_rotation[bone] - (pNextNext->bone_rotation[bone] - pStart->bone_rotation[bone]) * (1.0 / 6.0);
                
                interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, pStart->bone_rotation[bone], a_n_bone, b_n_bone, pEnd->bone_rotation[bone]);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    
    for(int frame = startKeyframe + 1; frame < inputLength; frame++) {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
        
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();
    int startKeyframe = 0;
    
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * pStart = pInputMotion->GetPosture(startKeyframe);
        Posture * pEnd = pInputMotion->GetPosture(endKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *pStart);
        pOutputMotion->SetPosture(endKeyframe, *pEnd);
        
        // interpolate in between
        for(int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            
            // interpolate root position
            interpolatedPosture.root_pos = pStart->root_pos * (1.0 - t) + pEnd->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double startAngles[3], endAngles[3], outAngles[3];
                
                // extract the Euler angles
                pStart->bone_rotation[bone].getValue(startAngles);
                pEnd->bone_rotation[bone].getValue(endAngles);
                
                // Convert Euler angles to Quaternions
                Quaternion<double> qStart, qEnd;
                Euler2Quaternion(startAngles, qStart);
                Euler2Quaternion(endAngles, qEnd);
                
                // Slerp
                Quaternion<double> qInterp = Slerp(t, qStart, qEnd);
                
                // Convert the interpolated Quaternion back to Euler angles
                Quaternion2Euler(qInterp, outAngles);
                
                // Save the result
                interpolatedPosture.bone_rotation[bone].setValue(outAngles);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    
    for(int frame = startKeyframe + 1; frame < inputLength; frame++) {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
            
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
    int inputLength = pInputMotion->GetNumFrames();
    int startKeyframe = 0;

    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
   
        int prevKeyframe = startKeyframe - (N + 1);
        if (prevKeyframe < 0) prevKeyframe = startKeyframe;
        
        int nextNextKeyframe = endKeyframe + (N + 1);
        if (nextNextKeyframe >= inputLength) nextNextKeyframe = endKeyframe;
        
        Posture * pPrev = pInputMotion->GetPosture(prevKeyframe);
        Posture * pStart = pInputMotion->GetPosture(startKeyframe);
        Posture * pEnd = pInputMotion->GetPosture(endKeyframe);
        Posture * pNextNext = pInputMotion->GetPosture(nextNextKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *pStart);
        pOutputMotion->SetPosture(endKeyframe, *pEnd);
        
        // interpolate in between
        for(int frame = 1; frame <= N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N + 1);
            
            // Calculate control points
            vector a_n_root = pStart->root_pos + (pEnd->root_pos -  pPrev->root_pos) * (1.0 / 6.0);
            vector b_n_root = pEnd->root_pos - (pNextNext->root_pos - pStart->root_pos) * (1.0 / 6.0);
            
            // interpolate root position
            interpolatedPosture.root_pos = DeCasteljauEuler(t, pStart->root_pos, a_n_root, b_n_root, pEnd->root_pos);
    
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
            {
                double prevAngles[3], startAngles[3], endAngles[3], nextNextAngles[3], a_angles[3], b_angles[3], outAngles[3];
                
                // extract the Euler angles
                pPrev->bone_rotation[bone].getValue(prevAngles);
                pStart->bone_rotation[bone].getValue(startAngles);
                pEnd->bone_rotation[bone].getValue(endAngles);
                pNextNext->bone_rotation[bone].getValue(nextNextAngles);
    
                // Calculate control points
                vector a_n_bone = pStart->bone_rotation[bone] + (pEnd->bone_rotation[bone] - pPrev->bone_rotation[bone]) * (1.0 / 6.0);
                vector b_n_bone = pEnd->bone_rotation[bone] - (pNextNext->bone_rotation[bone] - pStart->bone_rotation[bone]) * (1.0 / 6.0);
                
                // extract the Euler angles
                pStart->bone_rotation[bone].getValue(startAngles);
                a_n_bone.getValue(a_angles);
                b_n_bone.getValue(b_angles);
                pEnd->bone_rotation[bone].getValue(endAngles);
                
                // Convert Euler angles to Quaternions
                Quaternion<double> qStart, q_a, q_b, qEnd;
                Euler2Quaternion(startAngles, qStart);
                Euler2Quaternion(a_angles, q_a);
                Euler2Quaternion(b_angles, q_b);
                Euler2Quaternion(endAngles, qEnd);
                
                // Slerp
                Quaternion<double> qInterp = DeCasteljauQuaternion(t, qStart, q_a, q_b, qEnd);
                
                // Convert the interpolated Quaternion back to Euler angles
                Quaternion2Euler(qInterp, outAngles);
                
                // Save the result
                interpolatedPosture.bone_rotation[bone].setValue(outAngles);
            }
            
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        
        startKeyframe = endKeyframe;
    }
    
    for(int frame = startKeyframe + 1; frame < inputLength; frame++) {
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
    }
    
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
    double R[9];
    // Euler to rotation matrix
    Euler2Rotation(angles, R);
    // Rotation matrix to quaternion
    q = Quaternion<double>::Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
    double R[9];
    // Quaternion to rotation matrix
    q.Quaternion2Matrix(R);
    // Rotation matrix to euler
    Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd)
{
    Quaternion<double> q1 = qStart;
    Quaternion<double> q2 = qEnd;
    double weight1, weight2;
    
    double cosTheta = qStart.Gets()*qEnd.Gets() + qStart.Getx()*qEnd.Getx() + qStart.Gety()*qEnd.Gety() + qStart.Getz()*qEnd.Getz();
    
    // Clamp cosTheta to [-1, 1] to prevent acos() domain errors
    if (cosTheta > 1.0) cosTheta = 1.0;
    if (cosTheta < -1.0) cosTheta = -1.0;
    
    // Calculate theta and sin
    double theta = acos(cosTheta);
    double sinTheta = sin(theta);
    
    // Calculate weights
    if (sinTheta > 0.0001) {
        weight1 = sin((1.0 - t) * theta) / sinTheta;
        weight2 = sin(t * theta) / sinTheta;
    } else {
        // If angle close to 0, use LERP
        weight1 = 1.0 - t;
        weight2 = t;
    }
  Quaternion<double> result;
    result.Set(weight1 * q1.Gets() + weight2 * q2.Gets(),
               weight1 * q1.Getx() + weight2 * q2.Getx(),
               weight1 * q1.Gety() + weight2 * q2.Gety(),
               weight1 * q1.Getz() + weight2 * q2.Getz());
    
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
    // DeCasteljau on vetor
    vector q0 = p0 * (1.0 - t) + p1 * t;
    vector q1 = p1 * (1.0 - t) + p2 * t;
    vector q2 = p2 * (1.0 - t) + p3 * t;
    vector r0 = q0 * (1.0 - t) + q1 * t;
    vector r1 = q1 * (1.0 - t) + q2 * t;
    vector result = r0 * (1.0 - t) + r1 * t;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
    // DeCasteljau on Quaternion Sphere
    Quaternion<double> q0 = Slerp(t, p0, p1);
    Quaternion<double> q1 = Slerp(t, p1, p2);
    Quaternion<double> q2 = Slerp(t, p2, p3);
    Quaternion<double> r0 = Slerp(t, q0, q1);
    Quaternion<double> r1 = Slerp(t, q1, q2);
    Quaternion<double> result = Slerp(t, r0, r1);
  return result;
}

