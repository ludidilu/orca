namespace RVO
{
    public class UnitRect
    {
        private int m_XMin;
        private int m_YMin;
        private int m_Width;
        private int m_Height;

        public int height
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

        public int width
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

        public int x
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

        public int xMax
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

        public int xMin
        {
            get
            {
                return this.m_XMin;
            }
            set
            {
                int xMax = this.xMax;
                this.m_XMin = value;
                this.m_Width = xMax - this.m_XMin;
            }
        }

        public int y
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

        public int yMax
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

        public int yMin
        {
            get
            {
                return this.m_YMin;
            }
            set
            {
                int yMax = this.yMax;
                this.m_YMin = value;
                this.m_Height = yMax - this.m_YMin;
            }
        }

        public UnitRect(int x, int y, int width, int height)
        {
            this.m_XMin = x;
            this.m_YMin = y;
            this.m_Width = width;
            this.m_Height = height;
        }

        public bool Contains(int _x, int _y)
        {
            return _x >= this.xMin && _x < this.xMax && _y >= this.yMin && _y < this.yMax;
        }
    }
}
