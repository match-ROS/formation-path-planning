namespace general_types
{
    enum NeighborType
    {
        FourWay = 4,
        EightWay = 8
    };

    enum FreeNeighborMode
    {
        CostmapOnly = 0,
        CostmapAndMinimalCurveRadius = 1
    };
}