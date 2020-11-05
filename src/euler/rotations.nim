## Types and functions used to calculate rotation matrices and unit 
## quaternions given any sequence of fully specified euler angles

import math, strformat

type RotationSequence* = string
  ## A sequence of three principal axes (e.g., "xyz", "zyz", "yxz")
  ## about which rotations are performed.

type RotationAngles* = array[3, float]
  ## A sequence of three angles, each quantifying the angle of rotation
  ## about three sequential principal axes.

type RotationMatrix* = array[3, array[3, float]]
  ## A 3x3 rotation (direction cosine) matrix in row-major order.

type Quaternion* = array[4, float]
  ## Unit quaternion stored in w, x, y, z order.

type Order* = enum
  ## The order in which the sequence of principal axis rotations are
  ## performed.
  intrinsic, extrinsic

type Direction* = enum
  ## The direction of rotation, that is, what the rotation represents.
  active, passive

proc `[]`*(R: RotationMatrix, i: int, j: int): float =
  ## Operator to read elements from a rotation matrix.
  return R[i][j]

proc `[]=`*(R: var RotationMatrix, i: int, j: int, val: float) {.inline.} =
  ## Operator to write to elements of a rotation matrix.
  R[i][j] = val

func `*`(lhs: RotationMatrix, rhs: RotationMatrix): RotationMatrix =
  ## Operator to multiple two rotation matrices.
  for i in 0..<3:
    for j in 0..<3:
      for k in 0..<3:
        result[i, j] = result[i, j] + lhs[i, k] * rhs[k, j]

func `==`*(R1: RotationMatrix, R2: RotationMatrix,
    tolerance: float = 1e-6): bool =
  ## Operator to test the equivalency of two rotation matrices.
  for i in 0..<3:
    for j in 0..<3:
      if abs(R1[i, j] - R2[i, j]) > tolerance:
        return false
  return true

func `==`*(q1: Quaternion, q2: Quaternion, tolerance: float = 1e-6): bool =
  ## Operator to test the equivalency of two quaternions.
  for i in 0..<4:
    if abs(q1[i] - q2[i]) > tolerance:
      return false
  return true

func `$`*(R: RotationMatrix): string =
  result = "Rotation Matrix:"
  for i in 0..<3:
    result.add("\n")
    for j in 0..<3:
      if j != 0:
        result.add(" ")
      result.add(fmt"{R[i, j]:>7.4f}")

func `$`*(q: Quaternion): string =
  result = "Quaternion:\n"
  result.add(fmt" w: {q[0]:>7.4f}")
  result.add("\n")
  result.add(fmt" x: {q[1]:>7.4f}")
  result.add("\n")
  result.add(fmt" y: {q[2]:>7.4f}")
  result.add("\n")
  result.add(fmt" z: {q[3]:>7.4f}")

func isRotationSequenceValid*(rotSeq: RotationSequence): bool =
  ## Checks if a rotation sequence represents a valid sequence of three
  ## principal axes than can be used to represent any general rotation
  if rotSeq.len != 3:
    return false

  const validSeqs = ["xyz", "xzy", "yxz", "yzx",
                     "zxy", "zyx", "xyx", "xzx",
                     "yxy", "yzy", "zxz", "zyz"]
  return validSeqs.contains(rotSeq)

func transpose*(R: RotationMatrix): RotationMatrix =
  ## Calculates the transpose of a rotation matrix.
  result = [[R[0, 0], R[1, 0], R[2, 0]],
            [R[0, 1], R[1, 1], R[2, 1]],
            [R[0, 2], R[1, 2], R[2, 2]]]

func rotationX(angle: float): RotationMatrix =
  ## Given the angle of rotation, calculates the rotation matrix resulting
  ## from rotating by the angle about the x-axis.
  let c = cos(angle)
  let s = sin(angle)
  result = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]

func rotationY(angle: float): RotationMatrix =
  ## Given the angle of rotation, calculates the rotation matrix resulting
  ## from rotating by the angle about the y-axis.
  let c = cos(angle)
  let s = sin(angle)
  result = [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]

func rotationZ(angle: float): RotationMatrix =
  ## Given the angle of rotation, calculates the rotation matrix resulting
  ## from rotating by the angle about the z-axis.
  let c = cos(angle)
  let s = sin(angle)
  result = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]

func rotationActive(axis: char, angle: float): RotationMatrix =
  ## Given the angle and axis of rotation, calculates the rotation matrix
  ## resulting from rotating by the angle about the axis.
  case axis
  of 'x':
    result = rotationX(angle)
  of 'y':
    result = rotationY(angle)
  of 'z':
    result = rotationZ(angle)
  else:
    raise newException(ValueError, "Axis can only be x, y, or z")

func toRotationMatrix*(rotSeq: RotationSequence, rotAngles: RotationAngles,
    order: Order, direction: Direction): RotationMatrix =
  ## Calculates a rotation matrix given a sequence of three euler angles and
  ## the order and direction of the rotation
  if (order == intrinsic):
    result = rotationActive(rotSeq[0], rotAngles[0]) *
             rotationActive(rotSeq[1], rotAngles[1]) *
             rotationActive(rotSeq[2], rotAngles[2])
  else: # extrinsic
    result = rotationActive(rotSeq[2], rotAngles[2]) *
             rotationActive(rotSeq[1], rotAngles[1]) *
             rotationActive(rotSeq[0], rotAngles[0])

  if direction == passive:
    result = result.transpose()

func toQuaternion*(R: RotationMatrix): Quaternion =
  ## Converts a rotation matrix to a unit quaternion
  result[0] = sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0;

  result[1] = sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0;
  result[1] = result[1] * sgn(R[2, 1] - R[1, 2]).toFloat();

  result[2] = sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0;
  result[2] = result[2] * sgn(R[0, 2] - R[2, 0]).toFloat();

  result[3] = sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0;
  result[3] = result[3] * sgn(R[1, 0] - R[0, 1]).toFloat();
