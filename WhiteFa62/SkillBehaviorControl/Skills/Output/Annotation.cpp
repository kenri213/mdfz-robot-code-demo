/**
 * @file Annotation.cpp
 *
 * This file implements the Annotation skill.
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetSkillBehaviorControl.h"
#include "Debugging/Annotation.h"

option((WhiteFleetSkillBehaviorControl) Annotation,
       args((const std::string&) annotation),
       vars((std::string) lastAnnotationSent))
{
  initial_state(execute)
  {
    transition
    {
      if(annotation == lastAnnotationSent)
        goto done;
    }
    action
    {
      ANNOTATION("WhiteFleetBehavior", annotation);
      lastAnnotationSent = annotation;
    }
  }

  target_state(done)
  {
    transition
    {
      if(annotation != lastAnnotationSent)
        goto execute;
    }
  }
}
