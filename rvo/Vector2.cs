using System;
namespace RVO
{
    public struct Vector2
    {
        //
        // Fields
        //
        public double y;

        public double x;

        //
        // Static Properties
        //
        public static Vector2 down
        {
            get
            {
                return new Vector2(0, -1);
            }
        }

        public static Vector2 left
        {
            get
            {
                return new Vector2(-1, 0);
            }
        }

        public static Vector2 one
        {
            get
            {
                return new Vector2(1, 1);
            }
        }

        public static Vector2 right
        {
            get
            {
                return new Vector2(1, 0);
            }
        }

        public static Vector2 up
        {
            get
            {
                return new Vector2(0, 1);
            }
        }

        public static Vector2 zero
        {
            get
            {
                return new Vector2(0, 0);
            }
        }

        //
        // Properties
        //
        public double magnitude
        {
            get
            {
                return Math.Sqrt(this.x * this.x + this.y * this.y);
            }
        }

        public Vector2 normalized
        {
            get
            {
                Vector2 result = new Vector2(this.x, this.y);
                result.Normalize();
                return result;
            }
        }

        //
        // Indexer
        //
        public double this[int index]
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
        public Vector2(double x, double y)
        {
            this.x = x;
            this.y = y;
        }

        public static double Distance(Vector2 a, Vector2 b)
        {
            return (a - b).magnitude;
        }

        public static double Angle(Vector2 vector1, Vector2 vector2)
        {
            double sin = vector1.x * vector2.y - vector2.x * vector1.y;
            double cos = vector1.x * vector2.x + vector1.y * vector2.y;

            return Math.Abs(Math.Atan2(sin, cos));
        }

        public static double Dot(Vector2 lhs, Vector2 rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y;
        }

        public static double SqrMagnitude(Vector2 a)
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
            return this.x.Equals(vector.x) && this.y.Equals(vector.y);
        }

        public override int GetHashCode()
        {
            return this.x.GetHashCode() ^ this.y.GetHashCode() << 2;
        }

        public void Normalize()
        {
            double magnitude = this.magnitude;
            if (magnitude > 1E-05f)
            {
                this /= magnitude;
            }
            else
            {
                this = Vector2.zero;
            }
        }

        public void Set(double new_x, double new_y)
        {
            this.x = new_x;
            this.y = new_y;
        }

        public double SqrMagnitude()
        {
            return this.x * this.x + this.y * this.y;
        }

        //
        // Operators
        //
        public static Vector2 operator +(Vector2 a, Vector2 b)
        {
            return new Vector2(a.x + b.x, a.y + b.y);
        }

        public static Vector2 operator /(Vector2 a, double d)
        {
            return new Vector2(a.x / d, a.y / d);
        }

        public static bool operator ==(Vector2 lhs, Vector2 rhs)
        {
            return Vector2.SqrMagnitude(lhs - rhs) < 9.99999944E-11f;
        }

        public static bool operator !=(Vector2 lhs, Vector2 rhs)
        {
            return Vector2.SqrMagnitude(lhs - rhs) >= 9.99999944E-11f;
        }

        public static Vector2 operator *(double d, Vector2 a)
        {
            return new Vector2(a.x * d, a.y * d);
        }

        public static Vector2 operator *(Vector2 a, double d)
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
