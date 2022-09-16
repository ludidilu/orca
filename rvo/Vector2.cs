using System;
namespace RVO
{
    public struct Vector2
    {
        //
        // Fields
        //
        public float y;

        public float x;

        //
        // Static Properties
        //
        public static Vector2 Down
        {
            get
            {
                return new Vector2(0f, -1f);
            }
        }

        public static Vector2 Left
        {
            get
            {
                return new Vector2(-1f, 0f);
            }
        }

        public static Vector2 One
        {
            get
            {
                return new Vector2(1f, 1f);
            }
        }

        public static Vector2 Right
        {
            get
            {
                return new Vector2(1f, 0f);
            }
        }

        public static Vector2 Up
        {
            get
            {
                return new Vector2(0f, 1f);
            }
        }

        public static Vector2 Zero
        {
            get
            {
                return new Vector2(0f, 0f);
            }
        }

        //
        // Properties
        //
        public float magnitude
        {
            get
            {
                return (float)Math.Sqrt(x * x + y * y);
            }
        }

        public Vector2 normalized
        {
            get
            {
                Vector2 result = new Vector2(x, y);
                result.Normalize();
                return result;
            }
        }

        //
        // Indexer
        //
        public float this[int index]
        {
            get
            {
                if (index == 0)
                {
                    return this.x;
                }
                if (index != 1)
                {
                    throw new IndexOutOfRangeException("Invalid Vector2 index!");
                }
                return this.y;
            }
            set
            {
                if (index != 0)
                {
                    if (index != 1)
                    {
                        throw new IndexOutOfRangeException("Invalid Vector2 index!");
                    }
                    this.y = value;
                }
                else
                {
                    this.x = value;
                }
            }
        }

        //
        // Constructors
        //
        public Vector2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public static float Distance(Vector2 a, Vector2 b)
        {
            return (a - b).magnitude;
        }

        public static float Angle(Vector2 vector1, Vector2 vector2)
        {
            float sin = vector1.x * vector2.y - vector2.x * vector1.y;
            float cos = vector1.x * vector2.x + vector1.y * vector2.y;

            return Math.Abs((float)Math.Atan2(sin, cos));
        }

        public static float Dot(Vector2 lhs, Vector2 rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y;
        }

        public static float SqrMagnitude(Vector2 a)
        {
            return a.x * a.x + a.y * a.y;
        }

        //
        // Methods
        //
        public override bool Equals(object other)
        {
            if (!(other is Vector2))
            {
                return false;
            }
            Vector2 vector = (Vector2)other;
            return x.Equals(vector.x) && y.Equals(vector.y);
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() ^ y.GetHashCode() << 2;
        }

        public void Normalize()
        {
            float magnitude = this.magnitude;
            if (magnitude > 1E-05f)
            {
                this /= magnitude;
            }
            else
            {
                this = Zero;
            }
        }

        public void Set(float new_x, float new_y)
        {
            x = new_x;
            y = new_y;
        }

        public float SqrMagnitude()
        {
            return x * x + y * y;
        }

        //
        // Operators
        //
        public static Vector2 operator +(Vector2 a, Vector2 b)
        {
            return new Vector2(a.x + b.x, a.y + b.y);
        }

        public static Vector2 operator /(Vector2 a, float d)
        {
            return new Vector2(a.x / d, a.y / d);
        }

        public static bool operator ==(Vector2 lhs, Vector2 rhs)
        {
            return SqrMagnitude(lhs - rhs) < 9.99999944E-11f;
        }

        public static bool operator !=(Vector2 lhs, Vector2 rhs)
        {
            return SqrMagnitude(lhs - rhs) >= 9.99999944E-11f;
        }

        public static Vector2 operator *(float d, Vector2 a)
        {
            return new Vector2(a.x * d, a.y * d);
        }

        public static Vector2 operator *(Vector2 a, float d)
        {
            return new Vector2(a.x * d, a.y * d);
        }

        public static Vector2 operator -(Vector2 a, Vector2 b)
        {
            return new Vector2(a.x - b.x, a.y - b.y);
        }

        public static Vector2 operator -(Vector2 a)
        {
            return new Vector2(-a.x, -a.y);
        }
    }
}
