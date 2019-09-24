function RTW_SidParentMap() {
    this.sidParentMap = new Array();
    this.sidParentMap["end_point_filter:105"] = "end_point_filter";
    this.sidParentMap["end_point_filter:122"] = "end_point_filter";
    this.sidParentMap["end_point_filter:119"] = "end_point_filter";
    this.sidParentMap["end_point_filter:121"] = "end_point_filter";
    this.sidParentMap["end_point_filter:123"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:124"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:143"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:144"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:145"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:131"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:132"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:133"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:140"] = "end_point_filter:122";
    this.sidParentMap["end_point_filter:138"] = "end_point_filter:122";
    this.getParentSid = function(sid) { return this.sidParentMap[sid];}
}
    RTW_SidParentMap.instance = new RTW_SidParentMap();
