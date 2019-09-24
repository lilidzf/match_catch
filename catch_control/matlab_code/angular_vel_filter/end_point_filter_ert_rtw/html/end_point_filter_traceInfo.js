function RTW_Sid2UrlHash() {
	this.urlHashMap = new Array();
	/* <Root>/end_point */
	this.urlHashMap["end_point_filter:105"] = "ert_main.cpp:22&end_point_filter.cpp:844,914,984";
	/* <Root>/LTI filter
 */
	this.urlHashMap["end_point_filter:122"] = "end_point_filter.h:147";
	/* <Root>/end_point_filtered */
	this.urlHashMap["end_point_filter:121"] = "ert_main.cpp:25&end_point_filter.cpp:1053";
	/* <S1>/Median Filter */
	this.urlHashMap["end_point_filter:143"] = "end_point_filter.cpp:33,87,131,816,843&end_point_filter.h:41,42,43,44,45,46,47,48,49,50,51,52,53,80";
	/* <S1>/Median Filter1 */
	this.urlHashMap["end_point_filter:144"] = "end_point_filter.cpp:913&end_point_filter.h:54,55,56,57,58,59,60,61,62,63,64,65,66,81";
	/* <S1>/Median Filter2 */
	this.urlHashMap["end_point_filter:145"] = "end_point_filter.cpp:983&end_point_filter.h:67,68,69,70,71,72,73,74,75,76,77,78,79,82";
	/* <S1>/Rate Limiter */
	this.urlHashMap["end_point_filter:132"] = "end_point_filter.cpp:899,911,1071&end_point_filter.h:38";
	/* <S1>/Rate Limiter1 */
	this.urlHashMap["end_point_filter:133"] = "end_point_filter.cpp:969,981,1074&end_point_filter.h:39";
	/* <S1>/Rate Limiter2 */
	this.urlHashMap["end_point_filter:140"] = "end_point_filter.cpp:1039,1051,1077&end_point_filter.h:40";
	this.getUrlHash = function(sid) { return this.urlHashMap[sid];}
}
RTW_Sid2UrlHash.instance = new RTW_Sid2UrlHash();
function RTW_rtwnameSIDMap() {
	this.rtwnameHashMap = new Array();
	this.sidHashMap = new Array();
	this.rtwnameHashMap["<Root>"] = {sid: "end_point_filter"};
	this.sidHashMap["end_point_filter"] = {rtwname: "<Root>"};
	this.rtwnameHashMap["<S1>"] = {sid: "end_point_filter:122"};
	this.sidHashMap["end_point_filter:122"] = {rtwname: "<S1>"};
	this.rtwnameHashMap["<Root>/end_point"] = {sid: "end_point_filter:105"};
	this.sidHashMap["end_point_filter:105"] = {rtwname: "<Root>/end_point"};
	this.rtwnameHashMap["<Root>/LTI filter "] = {sid: "end_point_filter:122"};
	this.sidHashMap["end_point_filter:122"] = {rtwname: "<Root>/LTI filter "};
	this.rtwnameHashMap["<Root>/Sinal Specification"] = {sid: "end_point_filter:119"};
	this.sidHashMap["end_point_filter:119"] = {rtwname: "<Root>/Sinal Specification"};
	this.rtwnameHashMap["<Root>/end_point_filtered"] = {sid: "end_point_filter:121"};
	this.sidHashMap["end_point_filter:121"] = {rtwname: "<Root>/end_point_filtered"};
	this.rtwnameHashMap["<S1>/In1"] = {sid: "end_point_filter:123"};
	this.sidHashMap["end_point_filter:123"] = {rtwname: "<S1>/In1"};
	this.rtwnameHashMap["<S1>/Demux"] = {sid: "end_point_filter:124"};
	this.sidHashMap["end_point_filter:124"] = {rtwname: "<S1>/Demux"};
	this.rtwnameHashMap["<S1>/Median Filter"] = {sid: "end_point_filter:143"};
	this.sidHashMap["end_point_filter:143"] = {rtwname: "<S1>/Median Filter"};
	this.rtwnameHashMap["<S1>/Median Filter1"] = {sid: "end_point_filter:144"};
	this.sidHashMap["end_point_filter:144"] = {rtwname: "<S1>/Median Filter1"};
	this.rtwnameHashMap["<S1>/Median Filter2"] = {sid: "end_point_filter:145"};
	this.sidHashMap["end_point_filter:145"] = {rtwname: "<S1>/Median Filter2"};
	this.rtwnameHashMap["<S1>/Mux1"] = {sid: "end_point_filter:131"};
	this.sidHashMap["end_point_filter:131"] = {rtwname: "<S1>/Mux1"};
	this.rtwnameHashMap["<S1>/Rate Limiter"] = {sid: "end_point_filter:132"};
	this.sidHashMap["end_point_filter:132"] = {rtwname: "<S1>/Rate Limiter"};
	this.rtwnameHashMap["<S1>/Rate Limiter1"] = {sid: "end_point_filter:133"};
	this.sidHashMap["end_point_filter:133"] = {rtwname: "<S1>/Rate Limiter1"};
	this.rtwnameHashMap["<S1>/Rate Limiter2"] = {sid: "end_point_filter:140"};
	this.sidHashMap["end_point_filter:140"] = {rtwname: "<S1>/Rate Limiter2"};
	this.rtwnameHashMap["<S1>/Out1"] = {sid: "end_point_filter:138"};
	this.sidHashMap["end_point_filter:138"] = {rtwname: "<S1>/Out1"};
	this.getSID = function(rtwname) { return this.rtwnameHashMap[rtwname];}
	this.getRtwname = function(sid) { return this.sidHashMap[sid];}
}
RTW_rtwnameSIDMap.instance = new RTW_rtwnameSIDMap();
