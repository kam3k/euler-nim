when isMainModule:
  import euler/rotations
  import math
  import unittest

  const tolerance = 1e-6

  proc `==`(R1: RotationMatrix, R2: RotationMatrix): bool =
    for i in 0..<3:
      for j in 0..<3:
        if abs(R1[i, j] - R2[i, j]) > tolerance:
          return false
    return true

  proc `==`(q1: Quaternion, q2: Quaternion): bool =
    for i in 0..<4:
      if abs(q1[i] - q2[i]) > tolerance:
        return false
    return true

  suite "rotation sequences":

    test "no sequence":
      check(not isRotationSequenceValid(""))

    test "valid sequences":
      const validSeqs = ["xyz", "xzy", "yxz", "yzx",
                         "zxy", "zyx", "xyx", "xzx",
                         "yxy", "yzy", "zxz", "zyz"]
      for s in validSeqs:
        check(isRotationSequenceValid(s))

    test "invalid sequences":
      const invalidSeqs = ["xxz", "xyy", "zxx", "yyz", "xxx",
                           "yyy", "zzz", "zzx", "zzy", "yxx",
                           "xxy", "123", "1.23", "54", "sd9a"]
      for s in invalidSeqs:
        check(not isRotationSequenceValid(s))

  suite "identity rotations":
    const Rexpected = [[1.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0],
                       [0.0, 0.0, 1.0]]

    test "tait-bryant":
      var Ractual = toRotationMatrix("xyz", [0.0, 0.0, 0.0], intrinsic, passive)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("xyz", [0.0, 0.0, 0.0], intrinsic, active)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("xyz", [0.0, 0.0, 0.0], extrinsic, passive)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("xyz", [0.0, 0.0, 0.0], extrinsic, active)
      check(Ractual == Rexpected)

    test "euler":
      var Ractual = toRotationMatrix("yzy", [0.0, 0.0, 0.0], intrinsic, passive)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("yzy", [0.0, 0.0, 0.0], intrinsic, active)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("yzy", [0.0, 0.0, 0.0], extrinsic, passive)
      check(Ractual == Rexpected)
      Ractual = toRotationMatrix("yzy", [0.0, 0.0, 0.0], extrinsic, active)
      check(Ractual == Rexpected)

  suite "rotations about principal axes":
    const theta = PI / 4.3
    const s = sin(theta)
    const c = cos(theta)

    test "rotation about x-axis":
      const RexpectedActive = [[1.0, 0.0, 0.0],
                               [0.0, c, -s],
                               [0.0, s, c]]
      var Ractual = toRotationMatrix("xyz", [theta, 0.0, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("yxz", [0.0, theta, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("zyx", [0.0, 0.0, theta], intrinsic, active)
      check (Ractual == RexpectedActive)
      const RexpectedPassive = [[1.0, 0.0, 0.0],
                                [0.0, c, s],
                                [0.0, -s, c]]
      Ractual = toRotationMatrix("xyz", [theta, 0.0, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("yxz", [0.0, theta, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("zyx", [0.0, 0.0, theta], intrinsic, passive)
      check (Ractual == RexpectedPassive)

    test "rotation about y-axis":
      const RexpectedActive = [[c, 0.0, s],
                               [0.0, 1.0, 0.0],
                               [-s, 0.0, c]]
      var Ractual = toRotationMatrix("yzx", [theta, 0.0, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("xyz", [0.0, theta, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("zxy", [0.0, 0.0, theta], intrinsic, active)
      check (Ractual == RexpectedActive)
      const RexpectedPassive = [[c, 0.0, -s],
                                [0.0, 1.0, 0.0],
                                [s, 0.0, c]]
      Ractual = toRotationMatrix("yzx", [theta, 0.0, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("xyz", [0.0, theta, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("zxy", [0.0, 0.0, theta], intrinsic, passive)
      check (Ractual == RexpectedPassive)

    test "rotation about z-axis":
      const RexpectedActive = [[c, -s, 0.0],
                               [s, c, 0.0],
                               [0.0, 0.0, 1.0]]
      var Ractual = toRotationMatrix("zyx", [theta, 0.0, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("xzy", [0.0, theta, 0.0], intrinsic, active)
      check (Ractual == RexpectedActive)
      Ractual = toRotationMatrix("yxz", [0.0, 0.0, theta], intrinsic, active)
      check (Ractual == RexpectedActive)
      const RexpectedPassive = [[c, s, 0.0],
                                [-s, c, 0.0],
                                [0.0, 0.0, 1.0]]
      Ractual = toRotationMatrix("zyx", [theta, 0.0, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("xzy", [0.0, theta, 0.0], intrinsic, passive)
      check (Ractual == RexpectedPassive)
      Ractual = toRotationMatrix("yxz", [0.0, 0.0, theta], intrinsic, passive)
      check (Ractual == RexpectedPassive)

  suite "rotations of 90 degrees about each axis":
    type
      Vector = array[3, float]

    proc `*`(R: RotationMatrix, v: Vector): Vector =
      result[0] = R[0, 0]*v[0] + R[0, 1]*v[1] + R[0, 2]*v[2]
      result[1] = R[1, 0]*v[0] + R[1, 1]*v[1] + R[1, 2]*v[2]
      result[2] = R[2, 0]*v[0] + R[2, 1]*v[1] + R[2, 2]*v[2]

    proc `==`(v1: Vector, v2: Vector): bool =
      return abs(v1[0] - v2[0]) < tolerance and
             abs(v1[1] - v2[1]) < tolerance and
             abs(v1[2] - v2[2]) < tolerance

    const rotAngles = [PI/2.0, PI/2.0, PI/2.0]
    const a: Vector = [1.0, 0.0, 0.0]
    var bExpect: Vector
    var bActual: Vector

    test "intrinsic":
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("xyz", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("xyz", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("yzx", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("yzx", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, -1.0]
      bActual = toRotationMatrix("zyx", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("zyx", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("xzy", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bExpect = [0.0, -1.0, 0.0]
      bActual = toRotationMatrix("xzy", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("xyx", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("xyx", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("zxz", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("zxz", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [-1.0, 0.0, 0.0]
      bActual = toRotationMatrix("yzy", rotAngles, intrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("yzy", rotAngles, intrinsic, passive) * a
      check(bActual == bExpect)

    test "extrinsic":
      bExpect = [0.0, 0.0, -1.0]
      bActual = toRotationMatrix("xyz", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("xyz", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("yzx", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bExpect = [0.0, -1.0, 0.0]
      bActual = toRotationMatrix("yzx", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("zyx", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("zyx", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("xzy", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("xzy", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 1.0, 0.0]
      bActual = toRotationMatrix("xyx", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("xyx", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [0.0, 0.0, 1.0]
      bActual = toRotationMatrix("zxz", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("zxz", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)
      bExpect = [-1.0, 0.0, 0.0]
      bActual = toRotationMatrix("yzy", rotAngles, extrinsic, active) * a
      check(bActual == bExpect)
      bActual = toRotationMatrix("yzy", rotAngles, extrinsic, passive) * a
      check(bActual == bExpect)

  suite "general rotations":
    const rotAngles = [degToRad(11.0), degToRad(-38.0), degToRad(4.0)]

    test "intrinsic":
      const Rexpect = [[0.7860912, -0.0549689, -0.6156615],
                       [-0.0487127, 0.9874306, -0.1503595],
                       [0.616188, 0.1481869, 0.7735327]]

      var Ractual = toRotationMatrix("xyz", rotAngles, intrinsic, active)
      check(Ractual == Rexpect)

      # Order reveresed, but extrinsic should give same solution
      Ractual = toRotationMatrix("zyx",
                                 [rotAngles[2], rotAngles[1], rotAngles[0]],
                                 extrinsic, active)
      check(Ractual == Rexpect)

      # Transpose should give passive solution
      Ractual = toRotationMatrix("xyz", rotAngles, intrinsic, passive)
      check(Ractual == Rexpect.transpose())

    test "extrinsic":
      const Rexpect = [[0.786091, -0.185662, -0.589568],
                       [0.0549689, 0.971041, -0.232502],
                       [0.615661, 0.15036, 0.773533]]

      var Ractual = toRotationMatrix("xyz", rotAngles, extrinsic, active)
      check(Ractual == Rexpect)

      # Order reveresed, but extrinsic should give same solution
      Ractual = toRotationMatrix("zyx",
                                 [rotAngles[2], rotAngles[1], rotAngles[0]],
                                 intrinsic, active)
      check(Ractual == Rexpect)

      # Transpose should give passive solution
      Ractual = toRotationMatrix("xyz", rotAngles, extrinsic, passive)
      check(Ractual == Rexpect.transpose())

  suite "quaternions":
    test "convert rotation matrix to quaternion":
      # Identity
      const I = [[1.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0],
                 [0.0, 0.0, 1.0]]
      var qExpect = [1.0, 0.0, 0.0, 0.0]
      var qActual = I.toQuaternion()
      check(qActual == qExpect)

      # Random case verified with converter at
      # https://www.andre-gaschler.com/rotationconverter/
      const R = [[0.9590657, 0.2713668, 0.0809514],
                 [-0.1991297, 0.8495061, -0.4885557],
                 [-0.2013465, 0.4524372, 0.8687693]]
      qExpect = [0.9588197, 0.2453519, 0.0736056, -0.1226759]
      qActual = R.toQuaternion()
      check(qActual == qExpect)
