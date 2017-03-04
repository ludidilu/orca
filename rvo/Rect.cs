namespace RVO
{
    public struct Rect
    {
        private double m_XMin;
        private double m_YMin;
        private double m_Width;
        private double m_Height;

        public Vector2 center
        {
            get
            {
                return new Vector2(this.x + this.m_Width / 2, this.y + this.m_Height / 2);
            }
            set
            {
                this.m_XMin = value.x - this.m_Width / 2;
                this.m_YMin = value.y - this.m_Height / 2;
            }
        }

        public double height
        {
            get
            {
                return this.m_Height;
            }
            set
            {
                this.m_Height = value;
            }
        }

        public Vector2 max
        {
            get
            {
                return new Vector2(this.xMax, this.yMax);
            }
            set
            {
                this.xMax = value.x;
                this.yMax = value.y;
            }
        }

        public Vector2 min
        {
            get
            {
                return new Vector2(this.xMin, this.yMin);
            }
            set
            {
                this.xMin = value.x;
                this.yMin = value.y;
            }
        }

        public Vector2 position
        {
            get
            {
                return new Vector2(this.m_XMin, this.m_YMin);
            }
            set
            {
                this.m_XMin = value.x;
                this.m_YMin = value.y;
            }
        }

        public Vector2 size
        {
            get
            {
                return new Vector2(this.m_Width, this.m_Height);
            }
            set
            {
                this.m_Width = value.x;
                this.m_Height = value.y;
            }
        }

        public double width
        {
            get
            {
                return this.m_Width;
            }
            set
            {
                this.m_Width = value;
            }
        }

        public double x
        {
            get
            {
                return this.m_XMin;
            }
            set
            {
                this.m_XMin = value;
            }
        }

        public double xMax
        {
            get
            {
                return this.m_Width + this.m_XMin;
            }
            set
            {
                this.m_Width = value - this.m_XMin;
            }
        }

        public double xMin
        {
            get
            {
                return this.m_XMin;
            }
            set
            {
                double xMax = this.xMax;
                this.m_XMin = value;
                this.m_Width = xMax - this.m_XMin;
            }
        }

        public double y
        {
            get
            {
                return this.m_YMin;
            }
            set
            {
                this.m_YMin = value;
            }
        }

        public double yMax
        {
            get
            {
                return this.m_Height + this.m_YMin;
            }
            set
            {
                this.m_Height = value - this.m_YMin;
            }
        }

        public double yMin
        {
            get
            {
                return this.m_YMin;
            }
            set
            {
                double yMax = this.yMax;
                this.m_YMin = value;
                this.m_Height = yMax - this.m_YMin;
            }
        }

        //
        // Constructors
        //
        public Rect(Rect source)
        {
            this.m_XMin = source.m_XMin;
            this.m_YMin = source.m_YMin;
            this.m_Width = source.m_Width;
            this.m_Height = source.m_Height;
        }

        public Rect(Vector2 position, Vector2 size)
        {
            this.m_XMin = position.x;
            this.m_YMin = position.y;
            this.m_Width = size.x;
            this.m_Height = size.y;
        }

        public Rect(double x, double y, double width, double height)
        {
            this.m_XMin = x;
            this.m_YMin = y;
            this.m_Width = width;
            this.m_Height = height;
        }

        public bool Contains(Vector2 point)
        {
            return point.x >= this.xMin && point.x < this.xMax && point.y >= this.yMin && point.y < this.yMax;
        }

        public override bool Equals(object other)
        {
            if (!(other is Rect))
            {
                return false;
            }
            Rect rect = (Rect)other;
            return this.x.Equals(rect.x) && this.y.Equals(rect.y) && this.width.Equals(rect.width) && this.height.Equals(rect.height);
        }

        public override int GetHashCode()
        {
            return this.x.GetHashCode() ^ this.width.GetHashCode() << 2 ^ this.y.GetHashCode() >> 2 ^ this.height.GetHashCode() >> 1;
        }

        public void Set(double x, double y, double width, double height)
        {
            this.m_XMin = x;
            this.m_YMin = y;
            this.m_Width = width;
            this.m_Height = height;
        }

        //
        // Operators
        //
        public static bool operator ==(Rect lhs, Rect rhs)
        {
            return lhs.x == rhs.x && lhs.y == rhs.y && lhs.width == rhs.width && lhs.height == rhs.height;
        }

        public static bool operator !=(Rect lhs, Rect rhs)
        {
            return lhs.x != rhs.x || lhs.y != rhs.y || lhs.width != rhs.width || lhs.height != rhs.height;
        }
    }
}
