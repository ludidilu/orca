namespace RVO
{
    public struct Rect
    {
        private float m_XMin;
        private float m_YMin;
        private float m_Width;
        private float m_Height;

        public Vector2 center
        {
            get
            {
                return new Vector2(x + m_Width / 2, y + m_Height / 2);
            }
            set
            {
                m_XMin = value.x - m_Width / 2;
                m_YMin = value.y - m_Height / 2;
            }
        }

        public float height
        {
            get
            {
                return m_Height;
            }
            set
            {
                m_Height = value;
            }
        }

        public Vector2 max
        {
            get
            {
                return new Vector2(xMax, yMax);
            }
            set
            {
                xMax = value.x;
                yMax = value.y;
            }
        }

        public Vector2 min
        {
            get
            {
                return new Vector2(xMin, yMin);
            }
            set
            {
                xMin = value.x;
                yMin = value.y;
            }
        }

        public Vector2 position
        {
            get
            {
                return new Vector2(m_XMin, m_YMin);
            }
            set
            {
                m_XMin = value.x;
                m_YMin = value.y;
            }
        }

        public Vector2 size
        {
            get
            {
                return new Vector2(m_Width, m_Height);
            }
            set
            {
                m_Width = value.x;
                m_Height = value.y;
            }
        }

        public float width
        {
            get
            {
                return m_Width;
            }
            set
            {
                m_Width = value;
            }
        }

        public float x
        {
            get
            {
                return m_XMin;
            }
            set
            {
                m_XMin = value;
            }
        }

        public float xMax
        {
            get
            {
                return m_Width + m_XMin;
            }
            set
            {
                m_Width = value - m_XMin;
            }
        }

        public float xMin
        {
            get
            {
                return m_XMin;
            }
            set
            {
                float xMax = this.xMax;
                m_XMin = value;
                m_Width = xMax - m_XMin;
            }
        }

        public float y
        {
            get
            {
                return m_YMin;
            }
            set
            {
                m_YMin = value;
            }
        }

        public float yMax
        {
            get
            {
                return m_Height + m_YMin;
            }
            set
            {
                m_Height = value - m_YMin;
            }
        }

        public float yMin
        {
            get
            {
                return m_YMin;
            }
            set
            {
                float yMax = this.yMax;
                m_YMin = value;
                m_Height = yMax - m_YMin;
            }
        }

        //
        // Constructors
        //
        public Rect(Rect source)
        {
            m_XMin = source.m_XMin;
            m_YMin = source.m_YMin;
            m_Width = source.m_Width;
            m_Height = source.m_Height;
        }

        public Rect(Vector2 position, Vector2 size)
        {
            m_XMin = position.x;
            m_YMin = position.y;
            m_Width = size.x;
            m_Height = size.y;
        }

        public Rect(float x, float y, float width, float height)
        {
            m_XMin = x;
            m_YMin = y;
            m_Width = width;
            m_Height = height;
        }

        public bool Contains(Vector2 point)
        {
            return point.x >= xMin && point.x < xMax && point.y >= yMin && point.y < yMax;
        }

        public override bool Equals(object other)
        {
            if (!(other is Rect))
            {
                return false;
            }
            Rect rect = (Rect)other;
            return x.Equals(rect.x) && y.Equals(rect.y) && width.Equals(rect.width) && height.Equals(rect.height);
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() ^ width.GetHashCode() << 2 ^ y.GetHashCode() >> 2 ^ height.GetHashCode() >> 1;
        }

        public void Set(float x, float y, float width, float height)
        {
            m_XMin = x;
            m_YMin = y;
            m_Width = width;
            m_Height = height;
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
