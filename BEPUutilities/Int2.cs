using System;

namespace BEPUutilities
{
    /// <summary>
    /// Provides simple 2d cell hashing.
    /// </summary>
    public struct Int2(int x, int y) : IEquatable<Int2>
    {
        public int X = x;
        public int Y = y;

        public override readonly bool Equals(object obj)
            => Equals((Int2)obj);

        public readonly bool Equals(Int2 other)
            => X == other.X && Y == other.Y;

        public static bool operator ==(Int2 lhs, Int2 rhs)
            => lhs.X == rhs.X && lhs.Y == rhs.Y;

        public static bool operator !=(Int2 lhs, Int2 rhs)
        {
            return lhs.X != rhs.X || lhs.Y != rhs.Y;
        }

        public override readonly int GetHashCode()
            => (X * 533000401) ^ (Y * 920419813);

        public override readonly string ToString()
            => "{" + X + ", " + Y + "}";


    }
}
