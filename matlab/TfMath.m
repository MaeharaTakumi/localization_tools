classdef TfMath
    % check_tf_interpolation.m・check_tf_interpolation_ekf.m で使う四元数（[w x y z]）と時刻の計算
    methods (Static)
        function q = rpy2q(rpy)
            % roll, pitch, yaw（ZYX）→ [w x y z]
            cr = cos(rpy(1)/2); sr = sin(rpy(1)/2);
            cp = cos(rpy(2)/2); sp = sin(rpy(2)/2);
            cy = cos(rpy(3)/2); sy = sin(rpy(3)/2);
            q = [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy];
        end

        function yaw = q2yaw(q)
            yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
        end

        function q = qmul(a, b)
            q = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4), ...
                 a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3), ...
                 a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2), ...
                 a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
        end

        function v = qrot(q, p)
            % 点 p を q で回す
            r = TfMath.qmul(TfMath.qmul(q, [0 p]), [q(1) -q(2:4)]);
            v = r(2:4);
        end

        function a = qangle(q1, q2)
            % 2 つの回転の間の角度 [rad]（小さい角度でも精度が落ちない atan2 で）
            d = TfMath.qmul([q1(1) -q1(2:4)], q2);
            a = 2 * atan2(norm(d(2:4)), abs(d(1)));
        end

        function q = slerp(a, b, s)
            % 最短経路の球面線形補間（tf2::Quaternion::slerp と同じく内積が負なら b の符号を反転）
            if dot(a, b) < 0, b = -b; end
            th = acos(min(1, dot(a, b)));
            if th < 1e-12
                q = a;
            else
                q = (sin((1 - s)*th) * a + sin(s*th) * b) / sin(th);
            end
        end

        function q = nlerp(a, b, s)
            % 四元数を線形に混ぜて正規化
            if dot(a, b) < 0, b = -b; end
            q = (1 - s) * a + s * b;
            q = q / norm(q);
        end

        function [p, q] = compose(pA, qA, pB, qB)
            % T_A · T_B（A の座標で表した B）
            p = pA + TfMath.qrot(qA, pB);
            q = TfMath.qmul(qA, qB);
        end

        function st = stampMsg(sec, nanosec)
            % builtin_interfaces/Time
            st = ros2message("builtin_interfaces/Time");
            st.sec = int32(sec);
            st.nanosec = uint32(nanosec);
        end

        function st = stampFromNs(ns)
            % int64 のナノ秒 → builtin_interfaces/Time
            sec = idivide(int64(ns), int64(1e9), "floor");
            st = TfMath.stampMsg(sec, int64(ns) - sec * int64(1e9));
        end

        function v = lookup(tftree, parent, child, st)
            % parent → child を st の時刻で引いて [x y z w qx qy qz] にする（引けなければ NaN）
            v = nan(1, 7);
            try
                tf = getTransform(tftree, parent, child, st);
            catch
                return
            end
            t = tf.transform.translation;
            q = tf.transform.rotation;
            v = [t.x t.y t.z q.w q.x q.y q.z];
        end
    end
end
