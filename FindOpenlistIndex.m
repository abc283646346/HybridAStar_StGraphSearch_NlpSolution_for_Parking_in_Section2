function index = FindOpenlistIndex(openlist_, index0)
for index = 1 : size(openlist_,1)
    index_candidate = Convert3DConfigToIndex(openlist_(index,1:3));
    if (index_candidate == index0)
        return;
    end
end