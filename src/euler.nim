import euler/rotations
import math, os, parseopt, sequtils, strutils, sugar

proc writeHelp() =
  echo("""
Usage: euler [-r | --radians] [-e | --extrinsic ] [-p | --passive ]
             [-s S | --sequence:S] -- ANGLE ANGLE ANGLE

Calculates rotation matrix and quaternion for given Euler angle sequence.
The rotation matrix pre-multiplies vectors in a right-handed coordinate frame.
By default the sequence of angles are intepreted to be in degrees and are applied
in intrinsic order using the zyx sequence.

Examples:
   euler -- 20 -10 35
   euler -ep -- 11.1 23.9 -129.4
   euler -e -s yzy -- 41.2 -55.5 -97.8
   euler -p -s zxy -- -176.234 -0.231 44.399
   euler -rpe -s xzx -- 0.21 1.16 -2.81""")

when isMainModule:
  var p = initOptParser(commandLineParams())

  # Parse command line options
  var useRadians = false
  var order: Order = intrinsic
  var direction: Direction = active
  var sequence = "zyx"
  while true:
    p.next()
    case p.kind
    of cmdEnd: break
    of cmdShortOption, cmdLongOption:
      if p.kind == cmdLongOption and p.key == "": # look for "--"
        break
      elif p.key == "h" or p.key == "help":
        writeHelp()
        quit(0)
      elif p.key == "r" or p.key == "radians":
        useRadians = true
      elif p.key == "e" or p.key == "extrinsic":
        order = extrinsic
      elif p.key == "p" or p.key == "passive":
        direction = passive
      elif p.key == "s" or p.key == "sequence":
        sequence = p.val
      else:
        echo "Unrecognized argument: ", p.key, "\n"
        writeHelp()
        quit(QuitFailure)
    of cmdArgument:
      echo "Unrecognized argument: ", p.key, "\n"
      writeHelp()
      quit(QuitFailure)

  # Get rotation angles, ensure they're valid
  var rotAngles: RotationAngles
  try:
    let rotAnglesSeq = p.remainingArgs.map(x => parseFloat(x))
    if rotAnglesSeq.len != 3:
      raise newException(ValueError, "")
    rotAngles = [rotAnglesSeq[0], rotAnglesSeq[1], rotAnglesSeq[2]]
  except ValueError:
    echo "Invalid rotation angles: ", p.cmdLineRest(), "\n"
    writeHelp()
    quit(1)

  # Ensure sequence is valid
  if not isRotationSequenceValid(sequence):
    echo "Invalid rotation sequence: ", sequence, "\n"
    writeHelp()
    quit(1)

  # Convert angles to radians
  if not useRadians:
    rotAngles.apply(x => degToRad(x))

  # Create rotation matrix and quaternion from arguments
  let R = toRotationMatrix(sequence, rotAngles, order, direction)
  let q = toQuaternion(R)

  # Print results
  echo "\n", R
  echo "\n", q
