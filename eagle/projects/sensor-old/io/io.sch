<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.3.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="15" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="SamacSys_Parts">
<description>&lt;b&gt;https://componentsearchengine.com&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by SamacSys&lt;/author&gt;</description>
<packages>
<package name="ESP32-WROOM-32D">
<description>&lt;b&gt;ESP32-WROOM-32D-1&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-8.53655" y="8.2921" dx="2" dy="0.9" layer="1"/>
<smd name="2" x="-8.53655" y="7.0221" dx="2" dy="0.9" layer="1"/>
<smd name="3" x="-8.53655" y="5.7521" dx="2" dy="0.9" layer="1"/>
<smd name="4" x="-8.53655" y="4.4821" dx="2" dy="0.9" layer="1"/>
<smd name="5" x="-8.53655" y="3.2121" dx="2" dy="0.9" layer="1"/>
<smd name="6" x="-8.53655" y="1.9421" dx="2" dy="0.9" layer="1"/>
<smd name="7" x="-8.53655" y="0.6721" dx="2" dy="0.9" layer="1"/>
<smd name="8" x="-8.53655" y="-0.5979" dx="2" dy="0.9" layer="1"/>
<smd name="9" x="-8.53655" y="-1.8679" dx="2" dy="0.9" layer="1"/>
<smd name="10" x="-8.53655" y="-3.1379" dx="2" dy="0.9" layer="1"/>
<smd name="11" x="-8.53655" y="-4.4079" dx="2" dy="0.9" layer="1"/>
<smd name="12" x="-8.53655" y="-5.6779" dx="2" dy="0.9" layer="1"/>
<smd name="13" x="-8.53655" y="-6.9479" dx="2" dy="0.9" layer="1"/>
<smd name="14" x="-8.53655" y="-8.2179" dx="2" dy="0.9" layer="1"/>
<smd name="15" x="-5.75155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="16" x="-4.48155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="17" x="-3.21155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="18" x="-1.94155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="19" x="-0.67155" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="20" x="0.59845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="21" x="1.86845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="22" x="3.13845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="23" x="4.40845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="24" x="5.67845" y="-9.2179" dx="2" dy="0.9" layer="1" rot="R90"/>
<smd name="25" x="8.46345" y="-8.2179" dx="2" dy="0.9" layer="1"/>
<smd name="26" x="8.46345" y="-6.9479" dx="2" dy="0.9" layer="1"/>
<smd name="27" x="8.46345" y="-5.6779" dx="2" dy="0.9" layer="1"/>
<smd name="28" x="8.46345" y="-4.4079" dx="2" dy="0.9" layer="1"/>
<smd name="29" x="8.46345" y="-3.1379" dx="2" dy="0.9" layer="1"/>
<smd name="30" x="8.46345" y="-1.8679" dx="2" dy="0.9" layer="1"/>
<smd name="31" x="8.46345" y="-0.5979" dx="2" dy="0.9" layer="1"/>
<smd name="32" x="8.46345" y="0.6721" dx="2" dy="0.9" layer="1"/>
<smd name="33" x="8.46345" y="1.9421" dx="2" dy="0.9" layer="1"/>
<smd name="34" x="8.46345" y="3.2121" dx="2" dy="0.9" layer="1"/>
<smd name="35" x="8.46345" y="4.4821" dx="2" dy="0.9" layer="1"/>
<smd name="36" x="8.46345" y="5.7521" dx="2" dy="0.9" layer="1"/>
<smd name="37" x="8.46345" y="7.0221" dx="2" dy="0.9" layer="1"/>
<smd name="38" x="8.46345" y="8.2921" dx="2" dy="0.9" layer="1"/>
<smd name="39" x="-1.03655" y="0.7921" dx="5" dy="5" layer="1"/>
<wire x1="-9.03655" y1="15.7821" x2="8.96345" y2="15.7821" width="0.1" layer="51"/>
<wire x1="8.96345" y1="15.7821" x2="8.96345" y2="-9.7179" width="0.1" layer="51"/>
<wire x1="8.96345" y1="-9.7179" x2="-9.03655" y2="-9.7179" width="0.1" layer="51"/>
<wire x1="-9.03655" y1="-9.7179" x2="-9.03655" y2="15.7821" width="0.1" layer="51"/>
<wire x1="-9.03655" y1="-9.2179" x2="-9.03655" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="-9.7179" x2="-7.03655" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="6.96345" y1="-9.7179" x2="8.96345" y2="-9.7179" width="0.2" layer="21"/>
<wire x1="8.96345" y1="-9.7179" x2="8.96345" y2="-9.2179" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="9.5321" x2="-9.03655" y2="15.7821" width="0.2" layer="21"/>
<wire x1="-9.03655" y1="15.7821" x2="8.96345" y2="15.7821" width="0.2" layer="21"/>
<wire x1="8.96345" y1="15.7821" x2="8.96345" y2="9.5321" width="0.2" layer="21"/>
<text x="-2.92655" y="6.5921" size="1.27" layer="21">&gt;NAME</text>
<text x="-2.92655" y="4.0521" size="1.27" layer="21">&gt;VALUE</text>
<circle x="-6.63655" y="8.3321" radius="0.4242625" width="0" layer="21"/>
</package>
<package name="RAK811">
<smd name="1" x="-6.85" y="9" dx="2.2" dy="1.2" layer="1"/>
<smd name="2" x="-6.85" y="7.2" dx="2.2" dy="1.2" layer="1"/>
<smd name="3" x="-6.85" y="5.4" dx="2.2" dy="1.2" layer="1"/>
<smd name="4" x="-6.85" y="3.6" dx="2.2" dy="1.2" layer="1"/>
<smd name="5" x="-6.85" y="1.8" dx="2.2" dy="1.2" layer="1"/>
<smd name="6" x="-6.85" y="0" dx="2.2" dy="1.2" layer="1"/>
<smd name="7" x="-6.85" y="-1.8" dx="2.2" dy="1.2" layer="1"/>
<smd name="8" x="-6.85" y="-3.6" dx="2.2" dy="1.2" layer="1"/>
<smd name="9" x="-6.85" y="-5.4" dx="2.2" dy="1.2" layer="1"/>
<smd name="10" x="-6.85" y="-7.2" dx="2.2" dy="1.2" layer="1"/>
<smd name="11" x="-6.85" y="-9" dx="2.2" dy="1.2" layer="1"/>
<smd name="12" x="-4.5" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="13" x="-2.7" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="14" x="-0.9" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="15" x="0.9" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="16" x="2.7" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="17" x="4.5" y="-10.85" dx="2.2" dy="1.2" layer="1" rot="R90"/>
<smd name="18" x="6.85" y="-9" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="19" x="6.85" y="-7.2" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="20" x="6.85" y="-5.4" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="21" x="6.85" y="-3.6" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="22" x="6.85" y="-1.8" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="23" x="6.85" y="0" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="24" x="6.85" y="1.8" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="25" x="6.85" y="3.6" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="26" x="6.85" y="5.4" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="27" x="6.85" y="7.2" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="28" x="6.85" y="9" dx="2.2" dy="1.2" layer="1" rot="R180"/>
<smd name="29" x="4.5" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<smd name="30" x="2.7" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<smd name="31" x="0.9" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<smd name="32" x="-0.9" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<smd name="33" x="-2.7" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<smd name="34" x="-4.5" y="10.85" dx="2.2" dy="1.2" layer="1" rot="R270"/>
<wire x1="-7" y1="11" x2="7" y2="11" width="0.01" layer="51"/>
<wire x1="7" y1="11" x2="7" y2="-11" width="0.01" layer="51"/>
<wire x1="7" y1="-11" x2="-7" y2="-11" width="0.01" layer="51"/>
<wire x1="-7" y1="-11" x2="-7" y2="11" width="0.01" layer="51"/>
<wire x1="-5" y1="9" x2="5" y2="9" width="0.02" layer="21"/>
<wire x1="5" y1="9" x2="5" y2="-9" width="0.02" layer="21"/>
<wire x1="5" y1="-9" x2="-5" y2="-9" width="0.02" layer="21"/>
<wire x1="-5" y1="-9" x2="-5" y2="9" width="0.02" layer="21"/>
<circle x="-4.98655" y="9.0421" radius="0.4242625" width="0" layer="21"/>
<text x="-2.54" y="6.35" size="1.27" layer="21">&gt;NAME</text>
<text x="-2.54" y="7.62" size="1.27" layer="21">&gt;VALUE</text>
</package>
<package name="0603">
<description>&lt;b&gt;ERA3A&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<smd name="2" x="0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-0.8" y1="0.4" x2="0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="0.4" x2="0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="-0.4" x2="-0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-0.8" y1="-0.4" x2="-0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="-0.15" y1="0.4" x2="0.15" y2="0.4" width="0.2" layer="21"/>
<wire x1="0.15" y1="0.4" x2="0.15" y2="-0.4" width="0.2" layer="21"/>
<wire x1="0.15" y1="-0.4" x2="-0.15" y2="-0.4" width="0.2" layer="21"/>
<wire x1="-0.15" y1="-0.4" x2="-0.15" y2="0.4" width="0.2" layer="21"/>
</package>
<package name="0805">
<description>&lt;b&gt;GRM21B&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-0.83" y="0" dx="1.47" dy="0.97" layer="1" rot="R90"/>
<smd name="2" x="0.83" y="0" dx="1.47" dy="0.97" layer="1" rot="R90"/>
<text x="0.65" y="1.42" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-1" y1="0.625" x2="1" y2="0.625" width="0.1" layer="51"/>
<wire x1="1" y1="0.625" x2="1" y2="-0.625" width="0.1" layer="51"/>
<wire x1="1" y1="-0.625" x2="-1" y2="-0.625" width="0.1" layer="51"/>
<wire x1="-1" y1="-0.625" x2="-1" y2="0.625" width="0.1" layer="51"/>
<wire x1="-0.2" y1="0.6" x2="0.2" y2="0.6" width="0.2" layer="21"/>
<wire x1="0.2" y1="0.6" x2="0.2" y2="-0.6" width="0.2" layer="21"/>
<wire x1="0.2" y1="-0.6" x2="-0.2" y2="-0.6" width="0.2" layer="21"/>
<wire x1="-0.2" y1="-0.6" x2="-0.2" y2="0.6" width="0.2" layer="21"/>
</package>
<package name="JST-PH-2-2.0MM">
<pad name="1" x="-1" y="0" drill="0.9"/>
<pad name="2" x="1" y="0" drill="0.9"/>
<wire x1="-2.95" y1="1.7" x2="2.95" y2="1.7" width="0.2" layer="21"/>
<wire x1="2.95" y1="1.7" x2="2.95" y2="-2.8" width="0.2" layer="21"/>
<wire x1="2.95" y1="-2.8" x2="-2.95" y2="-2.8" width="0.2" layer="21"/>
<wire x1="-2.95" y1="-2.8" x2="-2.95" y2="1.7" width="0.2" layer="21"/>
<wire x1="-2.95" y1="1.7" x2="2.95" y2="1.7" width="0.1" layer="51"/>
<wire x1="2.95" y1="1.7" x2="2.95" y2="-2.8" width="0.1" layer="51"/>
<wire x1="2.95" y1="-2.8" x2="-2.95" y2="-2.8" width="0.1" layer="51"/>
<wire x1="-2.95" y1="-2.8" x2="-2.95" y2="1.7" width="0.1" layer="51"/>
<text x="-3.25" y="1.85" size="1.27" layer="51">&gt;NAME</text>
<text x="-3.25" y="3.05" size="1.27" layer="51">&gt;VALUE</text>
</package>
<package name="SOT23-3">
<description>&lt;b&gt;SOT-23 (TO-236)&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.05" y="0.95" dx="1.3" dy="0.6" layer="1"/>
<smd name="2" x="-1.05" y="-0.95" dx="1.3" dy="0.6" layer="1"/>
<smd name="3" x="1.05" y="0" dx="1.3" dy="0.6" layer="1"/>
<text x="0" y="0" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="0" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-0.65" y1="1.46" x2="0.65" y2="1.46" width="0.1" layer="51"/>
<wire x1="0.65" y1="1.46" x2="0.65" y2="-1.46" width="0.1" layer="51"/>
<wire x1="0.65" y1="-1.46" x2="-0.65" y2="-1.46" width="0.1" layer="51"/>
<wire x1="-0.65" y1="-1.46" x2="-0.65" y2="1.46" width="0.1" layer="51"/>
<wire x1="-0.05" y1="1.46" x2="0.05" y2="1.46" width="0.2" layer="21"/>
<wire x1="0.05" y1="1.46" x2="0.05" y2="-1.46" width="0.2" layer="21"/>
<wire x1="0.05" y1="-1.46" x2="-0.05" y2="-1.46" width="0.2" layer="21"/>
<wire x1="-0.05" y1="-1.46" x2="-0.05" y2="1.46" width="0.2" layer="21"/>
</package>
<package name="SOT-223">
<description>&lt;b&gt;3 Lead SOT-223 Plastic Package&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-3.35" y="2.29" dx="1.3" dy="0.95" layer="1"/>
<smd name="2" x="-3.35" y="0" dx="1.3" dy="0.95" layer="1"/>
<smd name="3" x="-3.35" y="-2.29" dx="1.3" dy="0.95" layer="1"/>
<smd name="4" x="3" y="0" dx="3.25" dy="2" layer="1" rot="R90"/>
<text x="0" y="1.4" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="1.4" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="2.35" y1="3.252" x2="2.35" y2="3.25" width="0.2" layer="21"/>
<wire x1="2.35" y1="3.25" x2="2.35" y2="-3.25" width="0.2" layer="21"/>
<wire x1="2.35" y1="-3.25" x2="2.35" y2="-3.252" width="0.2" layer="21"/>
<wire x1="-2.35" y1="-3.252" x2="-2.35" y2="-3.25" width="0.2" layer="21"/>
<wire x1="-2.35" y1="-3.25" x2="-2.35" y2="3.25" width="0.2" layer="21"/>
<wire x1="-2.35" y1="3.25" x2="-2.35" y2="3.252" width="0.2" layer="21"/>
<wire x1="-2.35" y1="-3.25" x2="2.35" y2="-3.25" width="0.2" layer="21"/>
<wire x1="-2.35" y1="3.25" x2="2.35" y2="3.25" width="0.2" layer="21"/>
<wire x1="-2.35" y1="3.25" x2="2.35" y2="3.25" width="0.1" layer="51"/>
<wire x1="2.35" y1="3.25" x2="2.35" y2="-3.25" width="0.1" layer="51"/>
<wire x1="2.35" y1="-3.25" x2="-2.35" y2="-3.25" width="0.1" layer="51"/>
<wire x1="-2.35" y1="-3.25" x2="-2.35" y2="3.25" width="0.1" layer="51"/>
<smd name="5" x="0.85" y="0" dx="6" dy="2.4" layer="1" rot="R90"/>
</package>
<package name="CON-8P-3_81-PLUG">
<pad name="2" x="-9.525" y="0" drill="1.4"/>
<pad name="3" x="-5.715" y="0" drill="1.4"/>
<pad name="4" x="-1.905" y="0" drill="1.4"/>
<pad name="5" x="1.905" y="0" drill="1.4"/>
<pad name="6" x="5.715" y="0" drill="1.4"/>
<pad name="7" x="9.525" y="0" drill="1.4"/>
<wire x1="-15.65" y1="-3" x2="15.65" y2="-3" width="0.1" layer="51"/>
<wire x1="15.65" y1="-3" x2="15.65" y2="4" width="0.1" layer="51"/>
<wire x1="15.65" y1="4" x2="-15.65" y2="4" width="0.1" layer="51"/>
<wire x1="-15.65" y1="4" x2="-15.65" y2="-3" width="0.1" layer="51"/>
<wire x1="-15.65" y1="4" x2="15.65" y2="4" width="0.2" layer="21"/>
<wire x1="15.65" y1="4" x2="15.65" y2="-3" width="0.2" layer="21"/>
<wire x1="15.65" y1="-3" x2="-15.65" y2="-3" width="0.2" layer="21"/>
<wire x1="-15.65" y1="-3" x2="-15.65" y2="4" width="0.2" layer="21"/>
<wire x1="-15.3" y1="-3.35" x2="15.3" y2="-3.35" width="0.1" layer="51"/>
<wire x1="15.3" y1="-3.35" x2="15.3" y2="4.8" width="0.1" layer="51"/>
<wire x1="15.3" y1="4.8" x2="-15.3" y2="4.8" width="0.1" layer="51"/>
<wire x1="-15.3" y1="4.8" x2="-15.3" y2="-3.35" width="0.1" layer="51"/>
<text x="-14.9" y="5.35" size="1.27" layer="51">&gt;NAME</text>
<text x="-14.9" y="6.75" size="1.27" layer="51">&gt;VALUE</text>
<pad name="8" x="13.335" y="0" drill="1.4"/>
<pad name="1" x="-13.335" y="0" drill="1.4"/>
</package>
<package name="SOT23-6">
<description>&lt;b&gt;5-Lead SOT-23&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.25" y="0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="2" x="-1.25" y="0" dx="1.2" dy="0.6" layer="1"/>
<smd name="3" x="-1.25" y="-0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="4" x="1.25" y="-0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="5" x="1.25" y="0" dx="1.2" dy="0.6" layer="1"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="1.27" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-0.85" y1="1.45" x2="0.85" y2="1.45" width="0.1" layer="51"/>
<wire x1="0.85" y1="1.45" x2="0.85" y2="-1.45" width="0.1" layer="51"/>
<wire x1="0.85" y1="-1.45" x2="-0.85" y2="-1.45" width="0.1" layer="51"/>
<wire x1="-0.85" y1="-1.45" x2="-0.85" y2="1.45" width="0.1" layer="51"/>
<wire x1="-0.85" y1="0.5" x2="0.1" y2="1.45" width="0.1" layer="51"/>
<wire x1="-0.3" y1="1.45" x2="0.3" y2="1.45" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.45" x2="0.3" y2="1.25" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.05" x2="0.3" y2="-1.45" width="0.2" layer="21"/>
<wire x1="0.3" y1="-1.45" x2="-0.3" y2="-1.45" width="0.2" layer="21"/>
<wire x1="-0.3" y1="-1.45" x2="-0.3" y2="1.05" width="0.2" layer="21"/>
<smd name="6" x="1.25" y="0.95" dx="1.2" dy="0.6" layer="1"/>
<wire x1="-0.3" y1="1.05" x2="-0.3" y2="1.25" width="0.2" layer="21"/>
<wire x1="-0.3" y1="1.25" x2="-0.3" y2="1.45" width="0.2" layer="21"/>
<wire x1="-0.3" y1="1.25" x2="0.3" y2="1.25" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.25" x2="0.3" y2="1.05" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.05" x2="-0.3" y2="1.05" width="0.2" layer="21"/>
</package>
<package name="DO-214AC-SMA">
<description>&lt;b&gt;DO-214AC (SMA)&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-2.1" y="0" dx="2.1" dy="1.5" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.1" dy="1.5" layer="1"/>
<text x="0" y="0" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="0" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-2.57" y1="1.28" x2="2.57" y2="1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="1.28" x2="2.57" y2="-1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="-1.28" x2="-2.57" y2="-1.28" width="0.1" layer="51"/>
<wire x1="-2.57" y1="-1.28" x2="-2.57" y2="1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="1.28" x2="-2.57" y2="1.28" width="0.2" layer="21"/>
<wire x1="-2.57" y1="-1.28" x2="2.57" y2="-1.28" width="0.2" layer="21"/>
</package>
<package name="DO-214AC-SMA-DIODE">
<description>&lt;b&gt;DO-214AC (SMA)&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-2.1" y="0" dx="2.1" dy="1.5" layer="1"/>
<smd name="2" x="2.1" y="0" dx="2.1" dy="1.5" layer="1"/>
<text x="0" y="0" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="0" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-2.57" y1="1.28" x2="2.57" y2="1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="1.28" x2="2.57" y2="-1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="-1.28" x2="-2.57" y2="-1.28" width="0.1" layer="51"/>
<wire x1="-2.57" y1="-1.28" x2="-2.57" y2="1.28" width="0.1" layer="51"/>
<wire x1="2.57" y1="1.28" x2="-2.57" y2="1.28" width="0.2" layer="21"/>
<wire x1="-2.57" y1="-1.28" x2="2.57" y2="-1.28" width="0.2" layer="21"/>
<wire x1="0.5" y1="0.6" x2="0.5" y2="-0.6" width="0.2" layer="21"/>
<wire x1="0.5" y1="-0.6" x2="-0.2" y2="0" width="0.2" layer="21"/>
<wire x1="-0.2" y1="0" x2="0.5" y2="0.6" width="0.2" layer="21"/>
<wire x1="-0.25" y1="0.6" x2="-0.25" y2="-0.6" width="0.2" layer="21"/>
</package>
<package name="SOD123-DIODE">
<description>&lt;b&gt;SOD123&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.635" y="0" dx="1.22" dy="0.91" layer="1" rot="R90"/>
<smd name="2" x="1.635" y="0" dx="1.22" dy="0.91" layer="1" rot="R90"/>
<text x="14.913" y="-3.073" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="14.913" y="-3.073" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-1.3375" y1="0.85" x2="1.3375" y2="0.85" width="0.1" layer="51"/>
<wire x1="1.3375" y1="0.85" x2="1.3375" y2="-0.85" width="0.1" layer="51"/>
<wire x1="1.3375" y1="-0.85" x2="-1.3375" y2="-0.85" width="0.1" layer="51"/>
<wire x1="-1.3375" y1="-0.85" x2="-1.3375" y2="0.85" width="0.1" layer="51"/>
<wire x1="-0.8" y1="0.85" x2="0.8" y2="0.85" width="0.2" layer="21"/>
<wire x1="-0.8" y1="-0.85" x2="0.8" y2="-0.85" width="0.2" layer="21"/>
<wire x1="0.45" y1="0.45" x2="0.45" y2="-0.45" width="0.2" layer="21"/>
<wire x1="0.45" y1="-0.45" x2="-0.35" y2="0" width="0.2" layer="21"/>
<wire x1="-0.35" y1="0" x2="0.45" y2="0.45" width="0.2" layer="21"/>
<wire x1="-0.35" y1="0.4" x2="-0.35" y2="-0.4" width="0.2" layer="21"/>
</package>
<package name="HEADER-8P-2R-DIP-2_0MM">
<pad name="1" x="-3" y="-1" drill="0.9"/>
<pad name="2" x="-1" y="-1" drill="0.9"/>
<wire x1="-4" y1="2" x2="4" y2="2" width="0.2" layer="21"/>
<wire x1="4" y1="2" x2="4" y2="-2" width="0.2" layer="21"/>
<wire x1="4" y1="-2" x2="-4" y2="-2" width="0.2" layer="21"/>
<wire x1="-4" y1="-2" x2="-4" y2="2" width="0.2" layer="21"/>
<wire x1="-4" y1="2" x2="4" y2="2" width="0.1" layer="51"/>
<wire x1="4" y1="2" x2="4" y2="-2" width="0.1" layer="51"/>
<wire x1="4" y1="-2" x2="-4" y2="-2" width="0.1" layer="51"/>
<wire x1="-4" y1="-2" x2="-4" y2="2" width="0.1" layer="51"/>
<text x="-3.7" y="2.2" size="1.27" layer="51">&gt;NAME</text>
<text x="-3.7" y="3.5" size="1.27" layer="51">&gt;VALUE</text>
<pad name="3" x="1" y="-1" drill="0.9"/>
<pad name="4" x="3" y="-1" drill="0.9"/>
<pad name="5" x="-3" y="1" drill="0.9"/>
<pad name="6" x="-1" y="1" drill="0.9"/>
<pad name="7" x="1" y="1" drill="0.9"/>
<pad name="8" x="3" y="1" drill="0.9"/>
</package>
<package name="SMA-RF">
<description>&lt;b&gt;32K201-400L5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="0" y="0" drill="1.6"/>
<pad name="2" x="-2.54" y="2.54" drill="1.6"/>
<pad name="3" x="-2.54" y="-2.54" drill="1.6"/>
<pad name="4" x="2.54" y="-2.54" drill="1.6"/>
<pad name="5" x="2.54" y="2.54" drill="1.6"/>
<text x="-0.424" y="0.7" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="-0.174" y="2" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-3.175" y1="3.175" x2="3.175" y2="3.175" width="0.1" layer="51"/>
<wire x1="3.175" y1="3.175" x2="3.175" y2="-3.175" width="0.1" layer="51"/>
<wire x1="3.175" y1="-3.175" x2="-3.175" y2="-3.175" width="0.1" layer="51"/>
<wire x1="-3.175" y1="-3.175" x2="-3.175" y2="3.175" width="0.1" layer="51"/>
<wire x1="-3.175" y1="1" x2="-3.175" y2="-1" width="0.2" layer="21"/>
<wire x1="-1" y1="3.175" x2="1" y2="3.175" width="0.2" layer="21"/>
<wire x1="-1" y1="-3.175" x2="1" y2="-3.175" width="0.2" layer="21"/>
<wire x1="3.175" y1="1" x2="3.175" y2="-1" width="0.2" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="ESP32-WROOM-32D">
<wire x1="5.08" y1="7.62" x2="48.26" y2="7.62" width="0.254" layer="94"/>
<wire x1="48.26" y1="-43.18" x2="48.26" y2="7.62" width="0.254" layer="94"/>
<wire x1="48.26" y1="-43.18" x2="5.08" y2="-43.18" width="0.254" layer="94"/>
<wire x1="5.08" y1="7.62" x2="5.08" y2="-43.18" width="0.254" layer="94"/>
<text x="6.35" y="8.636" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="6.35" y="10.414" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="GND_1" x="0" y="0" length="middle" direction="pwr"/>
<pin name="3V3" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="EN" x="0" y="-5.08" length="middle"/>
<pin name="IO36/SENSOR_VP" x="0" y="-7.62" length="middle"/>
<pin name="IO35/SENSOR_VN" x="0" y="-10.16" length="middle"/>
<pin name="IO34" x="0" y="-12.7" length="middle"/>
<pin name="IO35" x="0" y="-15.24" length="middle"/>
<pin name="IO32" x="0" y="-17.78" length="middle"/>
<pin name="IO33" x="0" y="-20.32" length="middle"/>
<pin name="IO25" x="0" y="-22.86" length="middle"/>
<pin name="IO26" x="0" y="-25.4" length="middle"/>
<pin name="IO27" x="0" y="-27.94" length="middle"/>
<pin name="IO14" x="0" y="-30.48" length="middle"/>
<pin name="IO12" x="0" y="-33.02" length="middle"/>
<pin name="GND_2" x="15.24" y="-48.26" length="middle" direction="pwr" rot="R90"/>
<pin name="IO13" x="17.78" y="-48.26" length="middle" rot="R90"/>
<pin name="SD2" x="20.32" y="-48.26" length="middle" rot="R90"/>
<pin name="SD3" x="22.86" y="-48.26" length="middle" rot="R90"/>
<pin name="CMD" x="25.4" y="-48.26" length="middle" rot="R90"/>
<pin name="CLK" x="27.94" y="-48.26" length="middle" rot="R90"/>
<pin name="SD0" x="30.48" y="-48.26" length="middle" rot="R90"/>
<pin name="SD1" x="33.02" y="-48.26" length="middle" rot="R90"/>
<pin name="IO15" x="35.56" y="-48.26" length="middle" rot="R90"/>
<pin name="IO2" x="38.1" y="-48.26" length="middle" rot="R90"/>
<pin name="IO0" x="53.34" y="-33.02" length="middle" rot="R180"/>
<pin name="IO4" x="53.34" y="-30.48" length="middle" rot="R180"/>
<pin name="IO16/RXD2" x="53.34" y="-27.94" length="middle" rot="R180"/>
<pin name="IO17/TXD2" x="53.34" y="-25.4" length="middle" rot="R180"/>
<pin name="IO5" x="53.34" y="-22.86" length="middle" rot="R180"/>
<pin name="IO18" x="53.34" y="-20.32" length="middle" rot="R180"/>
<pin name="IO19" x="53.34" y="-17.78" length="middle" rot="R180"/>
<pin name="NC" x="53.34" y="-15.24" length="middle" direction="nc" rot="R180"/>
<pin name="IO21/SDA" x="53.34" y="-12.7" length="middle" rot="R180"/>
<pin name="RXD0" x="53.34" y="-10.16" length="middle" rot="R180"/>
<pin name="TXD0" x="53.34" y="-7.62" length="middle" rot="R180"/>
<pin name="IO22/SCL" x="53.34" y="-5.08" length="middle" rot="R180"/>
<pin name="IO23" x="53.34" y="-2.54" length="middle" rot="R180"/>
<pin name="GND_3" x="53.34" y="0" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_4" x="53.34" y="5.08" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="RAK811">
<wire x1="5.08" y1="10.16" x2="38.1" y2="10.16" width="0.254" layer="94"/>
<wire x1="38.1" y1="-35.56" x2="38.1" y2="10.16" width="0.254" layer="94"/>
<wire x1="38.1" y1="-35.56" x2="5.08" y2="-35.56" width="0.254" layer="94"/>
<wire x1="5.08" y1="10.16" x2="5.08" y2="-35.56" width="0.254" layer="94"/>
<text x="6.35" y="11.176" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="6.35" y="12.954" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="GND_1" x="0" y="0" length="middle" direction="pwr"/>
<pin name="PB12" x="0" y="-2.54" length="middle"/>
<pin name="PB14" x="0" y="-5.08" length="middle"/>
<pin name="PB15" x="0" y="-7.62" length="middle"/>
<pin name="PA8" x="0" y="-10.16" length="middle"/>
<pin name="PA9/TX1" x="0" y="-12.7" length="middle"/>
<pin name="PA10/RX1" x="0" y="-15.24" length="middle"/>
<pin name="PA12/RTS1" x="0" y="-17.78" length="middle"/>
<pin name="PB4/CTS1" x="0" y="-20.32" length="middle"/>
<pin name="PA13" x="0" y="-22.86" length="middle"/>
<pin name="VCC" x="0" y="-25.4" length="middle" direction="pwr"/>
<pin name="GND_2" x="15.24" y="-40.64" length="middle" direction="pwr" rot="R90"/>
<pin name="PA14" x="17.78" y="-40.64" length="middle" rot="R90"/>
<pin name="PA15" x="20.32" y="-40.64" length="middle" rot="R90"/>
<pin name="PB3" x="22.86" y="-40.64" length="middle" rot="R90"/>
<pin name="PB5" x="25.4" y="-40.64" length="middle" rot="R90"/>
<pin name="BOOT0" x="27.94" y="-40.64" length="middle" rot="R90"/>
<pin name="PB8" x="43.18" y="-25.4" length="middle" rot="R180"/>
<pin name="PB9" x="43.18" y="-22.86" length="middle" rot="R180"/>
<pin name="PA2" x="43.18" y="-20.32" length="middle" rot="R180"/>
<pin name="GND_3" x="43.18" y="-17.78" length="middle" direction="pwr" rot="R180"/>
<pin name="PA1" x="43.18" y="-15.24" length="middle" rot="R180"/>
<pin name="PA0" x="43.18" y="-12.7" length="middle" rot="R180"/>
<pin name="RST" x="43.18" y="-10.16" length="middle" rot="R180"/>
<pin name="PB10" x="43.18" y="-7.62" length="middle" rot="R180"/>
<pin name="PB11" x="43.18" y="-5.08" length="middle" rot="R180"/>
<pin name="PB2" x="43.18" y="-2.54" length="middle" rot="R180"/>
<pin name="GND_4" x="43.18" y="0" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_5" x="27.94" y="15.24" length="middle" direction="pwr" rot="R270"/>
<pin name="GND_6" x="25.4" y="15.24" length="middle" direction="pwr" rot="R270"/>
<pin name="GND_7" x="22.86" y="15.24" length="middle" direction="pwr" rot="R270"/>
<pin name="GND_8" x="20.32" y="15.24" length="middle" direction="pwr" rot="R270"/>
<pin name="RF_OUT" x="17.78" y="15.24" length="middle" rot="R270"/>
<pin name="GND_9" x="15.24" y="15.24" length="middle" direction="pwr" rot="R270"/>
</symbol>
<symbol name="CAPACITOR">
<wire x1="5.588" y1="2.032" x2="5.588" y2="-2.032" width="0.254" layer="94"/>
<wire x1="7.112" y1="2.032" x2="7.112" y2="-2.032" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.588" y2="0" width="0.254" layer="94"/>
<wire x1="7.112" y1="0" x2="7.62" y2="0" width="0.254" layer="94"/>
<text x="3.81" y="3.048" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="4.826" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="middle" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="middle" direction="pas" rot="R180"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-3.048" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
<wire x1="-1.27" y1="-0.508" x2="1.27" y2="-0.508" width="0.254" layer="94"/>
<wire x1="-0.508" y1="-1.016" x2="0.508" y2="-1.016" width="0.254" layer="94"/>
</symbol>
<symbol name="VCC">
<circle x="0" y="0.762" radius="0.762" width="0.254" layer="94"/>
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="RESISTOR">
<text x="3.048" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.048" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<wire x1="5.08" y1="1.016" x2="5.842" y2="-1.016" width="0.254" layer="94"/>
<wire x1="5.842" y1="-1.016" x2="6.858" y2="1.016" width="0.254" layer="94"/>
<wire x1="6.858" y1="1.016" x2="7.874" y2="-1.016" width="0.254" layer="94"/>
<wire x1="7.874" y1="-1.016" x2="8.636" y2="1.016" width="0.254" layer="94"/>
<wire x1="8.636" y1="1.016" x2="9.652" y2="-1.016" width="0.254" layer="94"/>
<wire x1="9.652" y1="-1.016" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="1.016" x2="3.81" y2="-1.016" width="0.254" layer="94"/>
<wire x1="3.81" y1="-1.016" x2="3.048" y2="1.016" width="0.254" layer="94"/>
<wire x1="3.048" y1="1.016" x2="2.54" y2="0" width="0.254" layer="94"/>
</symbol>
<symbol name="CONNECTOR-2P">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-5.08" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
</symbol>
<symbol name="P-MOSFET">
<wire x1="5.08" y1="2.54" x2="3.302" y2="2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="4.064" x2="3.302" y2="4.064" width="0.254" layer="94"/>
<wire x1="3.302" y1="1.016" x2="5.08" y2="1.016" width="0.254" layer="94"/>
<wire x1="3.302" y1="4.572" x2="3.302" y2="3.556" width="0.254" layer="94"/>
<wire x1="3.302" y1="0.508" x2="3.302" y2="1.524" width="0.254" layer="94"/>
<wire x1="3.302" y1="2.032" x2="3.302" y2="3.048" width="0.254" layer="94"/>
<text x="5.588" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.588" y="1.778" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="G" x="0" y="5.08" visible="off" length="short"/>
<pin name="S" x="5.08" y="7.62" visible="off" length="short" rot="R270"/>
<pin name="D" x="5.08" y="-2.54" visible="off" length="short" rot="R90"/>
<polygon width="0.254" layer="94">
<vertex x="5.08" y="2.54"/>
<vertex x="4.064" y="3.048"/>
<vertex x="4.064" y="2.032"/>
</polygon>
<wire x1="5.08" y1="1.016" x2="5.08" y2="0" width="0.254" layer="94"/>
<wire x1="5.08" y1="5.08" x2="5.08" y2="2.54" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="0.508" width="0.254" layer="94"/>
</symbol>
<symbol name="VREG-SOT-223">
<wire x1="-17.78" y1="5.08" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-2.54" y2="5.08" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-17.78" y2="-7.62" width="0.254" layer="94"/>
<text x="-17.78" y="6.096" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="-17.78" y="7.874" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="IN" x="-22.86" y="0" length="middle" direction="pwr"/>
<pin name="GND" x="-10.16" y="-12.7" length="middle" direction="pwr" rot="R90"/>
<pin name="OUT" x="2.54" y="0" length="middle" direction="pwr" rot="R180"/>
<wire x1="-17.78" y1="5.08" x2="-17.78" y2="-7.62" width="0.254" layer="94"/>
</symbol>
<symbol name="CONNECTOR-8P">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-20.32" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-20.32" x2="5.08" y2="-20.32" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-20.32" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="0" y="-10.16" length="middle"/>
<pin name="6" x="0" y="-12.7" length="middle"/>
<pin name="7" x="0" y="-15.24" length="middle"/>
<pin name="8" x="0" y="-17.78" length="middle"/>
</symbol>
<symbol name="NL27WZ04DTT1G">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="A1" x="0" y="0" length="middle"/>
<pin name="GND" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="A2" x="0" y="-5.08" length="middle"/>
<pin name="-Y2" x="27.94" y="-5.08" length="middle" rot="R180"/>
<pin name="VCC" x="27.94" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="-Y1" x="27.94" y="0" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="TVS-DIODE">
<text x="1.524" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="1.524" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" rot="R180"/>
<wire x1="8.128" y1="1.016" x2="8.128" y2="0" width="0.254" layer="94"/>
<wire x1="8.128" y1="0" x2="8.128" y2="-1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
<wire x1="8.128" y1="1.016" x2="6.604" y2="0" width="0.254" layer="94"/>
<wire x1="6.604" y1="0" x2="8.128" y2="-1.016" width="0.254" layer="94"/>
<wire x1="4.572" y1="-1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="4.572" y1="0" x2="4.572" y2="1.016" width="0.254" layer="94"/>
<wire x1="4.572" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="4.572" y1="-1.016" x2="6.096" y2="0" width="0.254" layer="94"/>
<wire x1="6.096" y1="0" x2="4.572" y2="1.016" width="0.254" layer="94"/>
<wire x1="6.35" y1="0.508" x2="6.35" y2="-0.508" width="0.254" layer="94"/>
<wire x1="6.35" y1="-0.508" x2="6.858" y2="-1.016" width="0.254" layer="94"/>
<wire x1="6.35" y1="0.508" x2="5.842" y2="1.016" width="0.254" layer="94"/>
</symbol>
<symbol name="N-MOSFET">
<wire x1="5.08" y1="2.54" x2="3.302" y2="2.54" width="0.254" layer="94"/>
<wire x1="5.08" y1="1.016" x2="3.302" y2="1.016" width="0.254" layer="94"/>
<wire x1="3.302" y1="4.064" x2="5.08" y2="4.064" width="0.254" layer="94"/>
<wire x1="3.302" y1="0.508" x2="3.302" y2="1.524" width="0.254" layer="94"/>
<wire x1="3.302" y1="4.572" x2="3.302" y2="3.556" width="0.254" layer="94"/>
<wire x1="3.302" y1="3.048" x2="3.302" y2="2.032" width="0.254" layer="94"/>
<text x="5.588" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.588" y="1.778" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="G" x="0" y="5.08" visible="off" length="short"/>
<pin name="S" x="5.08" y="-2.54" visible="off" length="short" rot="R90"/>
<pin name="D" x="5.08" y="7.62" visible="off" length="short" rot="R270"/>
<wire x1="5.08" y1="4.064" x2="5.08" y2="5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.08" y2="2.54" width="0.254" layer="94"/>
<wire x1="2.54" y1="5.08" x2="2.54" y2="0.508" width="0.254" layer="94"/>
<polygon width="0.254" layer="94">
<vertex x="3.81" y="2.54"/>
<vertex x="4.826" y="3.048"/>
<vertex x="4.826" y="2.032"/>
</polygon>
</symbol>
<symbol name="FUSE">
<wire x1="2.54" y1="1.016" x2="10.16" y2="1.016" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.016" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="1.016" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.016" x2="2.54" y2="-1.016" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.016" x2="2.54" y2="0" width="0.254" layer="94"/>
<text x="2.54" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="2.54" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-1.016" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="6.35" y2="0" width="0.1524" layer="94" curve="-126.869898"/>
<wire x1="6.35" y1="0" x2="7.62" y2="0" width="0.1524" layer="94" curve="126.870349"/>
<wire x1="5.08" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<wire x1="7.62" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
</symbol>
<symbol name="VEXT">
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VEXT" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
<wire x1="-1.016" y1="1.524" x2="1.016" y2="1.524" width="0.254" layer="94"/>
<wire x1="1.016" y1="1.524" x2="1.016" y2="0" width="0.254" layer="94"/>
<wire x1="1.016" y1="0" x2="-1.016" y2="0" width="0.254" layer="94"/>
<wire x1="-1.016" y1="0" x2="-1.016" y2="1.524" width="0.254" layer="94"/>
</symbol>
<symbol name="DIODE">
<text x="1.524" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="1.524" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="K" x="0" y="0" visible="off" length="short"/>
<pin name="A" x="12.7" y="0" visible="off" length="short" rot="R180"/>
<wire x1="4.572" y1="0" x2="8.128" y2="1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="1.016" x2="8.128" y2="0" width="0.254" layer="94"/>
<wire x1="8.128" y1="0" x2="8.128" y2="-1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="-1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="4.572" y1="1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="4.572" y1="0" x2="4.572" y2="-1.016" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="4.572" y2="0" width="0.1524" layer="94"/>
<wire x1="8.128" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
</symbol>
<symbol name="ZENER">
<text x="1.524" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="1.524" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="K" x="0" y="0" visible="off" length="short"/>
<pin name="A" x="12.7" y="0" visible="off" length="short" rot="R180"/>
<wire x1="4.572" y1="0" x2="8.128" y2="1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="1.016" x2="8.128" y2="0" width="0.254" layer="94"/>
<wire x1="8.128" y1="0" x2="8.128" y2="-1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="-1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="4.572" y2="0" width="0.1524" layer="94"/>
<wire x1="8.128" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
<wire x1="4.572" y1="0.508" x2="4.572" y2="-0.508" width="0.254" layer="94"/>
<wire x1="4.572" y1="-0.508" x2="5.08" y2="-1.016" width="0.254" layer="94"/>
<wire x1="4.572" y1="0.508" x2="4.064" y2="1.016" width="0.254" layer="94"/>
</symbol>
<symbol name="SCHOTTKY-SERIE-3P">
<text x="-3.556" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="-2.54" y="-12.7" visible="off" length="short" rot="R90"/>
<wire x1="-2.54" y1="-4.572" x2="-1.524" y2="-8.128" width="0.254" layer="94"/>
<wire x1="-1.524" y1="-8.128" x2="-2.54" y2="-8.128" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-8.128" x2="-3.556" y2="-8.128" width="0.254" layer="94"/>
<wire x1="-3.556" y1="-8.128" x2="-2.54" y2="-4.572" width="0.254" layer="94"/>
<wire x1="-1.524" y1="-4.572" x2="-2.54" y2="-4.572" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-4.572" x2="-3.556" y2="-4.572" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-8.128" x2="-2.54" y2="-10.16" width="0.1524" layer="94"/>
<pin name="2" x="2.54" y="-12.7" visible="off" length="short" rot="R90"/>
<wire x1="2.54" y1="-8.128" x2="1.524" y2="-4.572" width="0.254" layer="94"/>
<wire x1="1.524" y1="-4.572" x2="2.54" y2="-4.572" width="0.254" layer="94"/>
<wire x1="2.54" y1="-4.572" x2="3.556" y2="-4.572" width="0.254" layer="94"/>
<wire x1="3.556" y1="-4.572" x2="2.54" y2="-8.128" width="0.254" layer="94"/>
<wire x1="1.524" y1="-8.128" x2="2.54" y2="-8.128" width="0.254" layer="94"/>
<wire x1="2.54" y1="-8.128" x2="3.556" y2="-8.128" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-4.572" x2="-2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-10.16" x2="2.54" y2="-8.128" width="0.1524" layer="94"/>
<pin name="3" x="0" y="0" visible="off" length="short" rot="R270"/>
<wire x1="-3.556" y1="-4.572" x2="-3.556" y2="-4.064" width="0.254" layer="94"/>
<wire x1="-3.556" y1="-4.064" x2="-3.048" y2="-4.064" width="0.254" layer="94"/>
<wire x1="-1.524" y1="-4.572" x2="-1.524" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-1.524" y1="-5.08" x2="-2.032" y2="-5.08" width="0.254" layer="94"/>
<wire x1="3.556" y1="-8.128" x2="3.556" y2="-8.636" width="0.254" layer="94"/>
<wire x1="3.556" y1="-8.636" x2="3.048" y2="-8.636" width="0.254" layer="94"/>
<wire x1="1.524" y1="-8.128" x2="1.524" y2="-7.62" width="0.254" layer="94"/>
<wire x1="1.524" y1="-7.62" x2="2.032" y2="-7.62" width="0.254" layer="94"/>
<wire x1="2.54" y1="-4.572" x2="2.54" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="-2.54" y2="-2.54" width="0.1524" layer="94"/>
<text x="-3.556" y="1.016" size="1.778" layer="95">&gt;NAME</text>
</symbol>
<symbol name="HALL-SENSOR">
<wire x1="-17.78" y1="7.62" x2="-2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-2.54" y2="7.62" width="0.254" layer="94"/>
<wire x1="-2.54" y1="-7.62" x2="-17.78" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-17.78" y1="7.62" x2="-17.78" y2="2.54" width="0.254" layer="94"/>
<text x="-17.78" y="8.636" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="-17.78" y="10.414" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="VDD" x="-10.16" y="12.7" length="middle" direction="pwr" rot="R270"/>
<pin name="VSS" x="-10.16" y="-12.7" length="middle" direction="pwr" rot="R90"/>
<pin name="OUT" x="2.54" y="0" length="middle" rot="R180"/>
<wire x1="-17.78" y1="2.54" x2="-17.78" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-17.78" y1="-2.54" x2="-17.78" y2="-7.62" width="0.254" layer="94"/>
<wire x1="-17.78" y1="2.54" x2="-12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="-12.7" y1="2.54" x2="-12.7" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-2.54" x2="-17.78" y2="-2.54" width="0.254" layer="94"/>
<wire x1="-17.78" y1="-2.54" x2="-12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="-17.78" y1="2.54" x2="-12.7" y2="-2.54" width="0.254" layer="94"/>
</symbol>
<symbol name="CONNECTOR-8P-2R">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-10.16" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-10.16" x2="5.08" y2="-10.16" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-10.16" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="6" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="7" x="17.78" y="-5.08" length="middle" rot="R180"/>
<pin name="8" x="17.78" y="-7.62" length="middle" rot="R180"/>
</symbol>
<symbol name="ANT">
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="ANT" x="0" y="-2.54" visible="off" length="short" rot="R90"/>
<wire x1="0" y1="0" x2="-1.016" y2="1.016" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="1.016" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<pin name="GND" x="2.54" y="-2.54" visible="off" length="short" direction="pwr" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ESP32-WROOM-32D" prefix="MOD" uservalue="yes">
<gates>
<gate name="G$1" symbol="ESP32-WROOM-32D" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ESP32-WROOM-32D">
<connects>
<connect gate="G$1" pin="3V3" pad="2"/>
<connect gate="G$1" pin="CLK" pad="20"/>
<connect gate="G$1" pin="CMD" pad="19"/>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND_1" pad="1"/>
<connect gate="G$1" pin="GND_2" pad="15"/>
<connect gate="G$1" pin="GND_3" pad="38"/>
<connect gate="G$1" pin="GND_4" pad="39"/>
<connect gate="G$1" pin="IO0" pad="25"/>
<connect gate="G$1" pin="IO12" pad="14"/>
<connect gate="G$1" pin="IO13" pad="16"/>
<connect gate="G$1" pin="IO14" pad="13"/>
<connect gate="G$1" pin="IO15" pad="23"/>
<connect gate="G$1" pin="IO16/RXD2" pad="27"/>
<connect gate="G$1" pin="IO17/TXD2" pad="28"/>
<connect gate="G$1" pin="IO18" pad="30"/>
<connect gate="G$1" pin="IO19" pad="31"/>
<connect gate="G$1" pin="IO2" pad="24"/>
<connect gate="G$1" pin="IO21/SDA" pad="33"/>
<connect gate="G$1" pin="IO22/SCL" pad="36"/>
<connect gate="G$1" pin="IO23" pad="37"/>
<connect gate="G$1" pin="IO25" pad="10"/>
<connect gate="G$1" pin="IO26" pad="11"/>
<connect gate="G$1" pin="IO27" pad="12"/>
<connect gate="G$1" pin="IO32" pad="8"/>
<connect gate="G$1" pin="IO33" pad="9"/>
<connect gate="G$1" pin="IO34" pad="6"/>
<connect gate="G$1" pin="IO35" pad="7"/>
<connect gate="G$1" pin="IO35/SENSOR_VN" pad="5"/>
<connect gate="G$1" pin="IO36/SENSOR_VP" pad="4"/>
<connect gate="G$1" pin="IO4" pad="26"/>
<connect gate="G$1" pin="IO5" pad="29"/>
<connect gate="G$1" pin="NC" pad="32"/>
<connect gate="G$1" pin="RXD0" pad="34"/>
<connect gate="G$1" pin="SD0" pad="21"/>
<connect gate="G$1" pin="SD1" pad="22"/>
<connect gate="G$1" pin="SD2" pad="17"/>
<connect gate="G$1" pin="SD3" pad="18"/>
<connect gate="G$1" pin="TXD0" pad="35"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RAK811" prefix="MOD" uservalue="yes">
<gates>
<gate name="G$1" symbol="RAK811" x="0" y="0"/>
</gates>
<devices>
<device name="" package="RAK811">
<connects>
<connect gate="G$1" pin="BOOT0" pad="17"/>
<connect gate="G$1" pin="GND_1" pad="1"/>
<connect gate="G$1" pin="GND_2" pad="12"/>
<connect gate="G$1" pin="GND_3" pad="21"/>
<connect gate="G$1" pin="GND_4" pad="28"/>
<connect gate="G$1" pin="GND_5" pad="29"/>
<connect gate="G$1" pin="GND_6" pad="30"/>
<connect gate="G$1" pin="GND_7" pad="31"/>
<connect gate="G$1" pin="GND_8" pad="32"/>
<connect gate="G$1" pin="GND_9" pad="34"/>
<connect gate="G$1" pin="PA0" pad="23"/>
<connect gate="G$1" pin="PA1" pad="22"/>
<connect gate="G$1" pin="PA10/RX1" pad="7"/>
<connect gate="G$1" pin="PA12/RTS1" pad="8"/>
<connect gate="G$1" pin="PA13" pad="10"/>
<connect gate="G$1" pin="PA14" pad="13"/>
<connect gate="G$1" pin="PA15" pad="14"/>
<connect gate="G$1" pin="PA2" pad="20"/>
<connect gate="G$1" pin="PA8" pad="5"/>
<connect gate="G$1" pin="PA9/TX1" pad="6"/>
<connect gate="G$1" pin="PB10" pad="25"/>
<connect gate="G$1" pin="PB11" pad="26"/>
<connect gate="G$1" pin="PB12" pad="2"/>
<connect gate="G$1" pin="PB14" pad="3"/>
<connect gate="G$1" pin="PB15" pad="4"/>
<connect gate="G$1" pin="PB2" pad="27"/>
<connect gate="G$1" pin="PB3" pad="15"/>
<connect gate="G$1" pin="PB4/CTS1" pad="9"/>
<connect gate="G$1" pin="PB5" pad="16"/>
<connect gate="G$1" pin="PB8" pad="18"/>
<connect gate="G$1" pin="PB9" pad="19"/>
<connect gate="G$1" pin="RF_OUT" pad="33"/>
<connect gate="G$1" pin="RST" pad="24"/>
<connect gate="G$1" pin="VCC" pad="11"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAPACITOR-0603" prefix="C" uservalue="yes">
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="GND" prefix="GND" uservalue="yes">
<gates>
<gate name="1" symbol="GND" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VCC" prefix="SUP" uservalue="yes">
<gates>
<gate name="G$1" symbol="VCC" x="0" y="0"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RESISTOR-0603" prefix="R" uservalue="yes">
<gates>
<gate name="G$1" symbol="RESISTOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CAPACITOR-0805" prefix="C" uservalue="yes">
<gates>
<gate name="G$1" symbol="CAPACITOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="JST-PH-2-2.0MM" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-2P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="JST-PH-2-2.0MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="P-MOSFET" prefix="Q" uservalue="yes">
<gates>
<gate name="G$1" symbol="P-MOSFET" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-3">
<connects>
<connect gate="G$1" pin="D" pad="3"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VREG-SOT-223" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="VREG-SOT-223" x="22.86" y="0"/>
</gates>
<devices>
<device name="" package="SOT-223">
<connects>
<connect gate="G$1" pin="GND" pad="1"/>
<connect gate="G$1" pin="IN" pad="3"/>
<connect gate="G$1" pin="OUT" pad="2 4 5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CON-89-3_81-PLUG" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-8P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CON-8P-3_81-PLUG">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="NL27WZ04DTT1G" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="NL27WZ04DTT1G" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-6">
<connects>
<connect gate="G$1" pin="-Y1" pad="6"/>
<connect gate="G$1" pin="-Y2" pad="4"/>
<connect gate="G$1" pin="A1" pad="1"/>
<connect gate="G$1" pin="A2" pad="3"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="VCC" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="TVS-DIODE-SMA" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="TVS-DIODE" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DO-214AC-SMA">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="RESISTOR-0805" prefix="R" uservalue="yes">
<gates>
<gate name="G$1" symbol="RESISTOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="N-MOSFET" prefix="Q" uservalue="yes">
<gates>
<gate name="G$1" symbol="N-MOSFET" x="0" y="-5.08"/>
</gates>
<devices>
<device name="" package="SOT23-3">
<connects>
<connect gate="G$1" pin="D" pad="3"/>
<connect gate="G$1" pin="G" pad="1"/>
<connect gate="G$1" pin="S" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="FUSE-0805" prefix="F" uservalue="yes">
<gates>
<gate name="G$1" symbol="FUSE" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0805">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="VEXT">
<gates>
<gate name="G$1" symbol="VEXT" x="0" y="2.54"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="DIODE-DO-214AC-SMA" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="DIODE" x="0" y="0"/>
</gates>
<devices>
<device name="" package="DO-214AC-SMA-DIODE">
<connects>
<connect gate="G$1" pin="A" pad="2"/>
<connect gate="G$1" pin="K" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="ZENER-SOD123" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="ZENER" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOD123-DIODE">
<connects>
<connect gate="G$1" pin="A" pad="2"/>
<connect gate="G$1" pin="K" pad="1"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="SCHOTTKY-SERIE-3P" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="SCHOTTKY-SERIE-3P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-3">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="S-5712-SOT23-3" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="HALL-SENSOR" x="-2.54" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-3">
<connects>
<connect gate="G$1" pin="OUT" pad="2"/>
<connect gate="G$1" pin="VDD" pad="1"/>
<connect gate="G$1" pin="VSS" pad="3"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="HEADER-8P-2R-DIP-2_0MM" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-8P-2R" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HEADER-8P-2R-DIP-2_0MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="SMA-RF" prefix="ANT" uservalue="yes">
<gates>
<gate name="G$1" symbol="ANT" x="0" y="2.54"/>
</gates>
<devices>
<device name="" package="SMA-RF">
<connects>
<connect gate="G$1" pin="ANT" pad="1"/>
<connect gate="G$1" pin="GND" pad="2 3 4 5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="MOD1" library="SamacSys_Parts" deviceset="ESP32-WROOM-32D" device=""/>
<part name="MOD2" library="SamacSys_Parts" deviceset="RAK811" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C3" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND6" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND7" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND8" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND9" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND10" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R21" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R19" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP3" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP4" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="C9" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND12" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP5" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R24" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C11" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="SUP6" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND13" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND15" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R18" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R20" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R17" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R16" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP7" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP8" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND16" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP9" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND17" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="CON1" library="SamacSys_Parts" deviceset="JST-PH-2-2.0MM" device=""/>
<part name="GND20" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="Q1" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="R5" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C4" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND21" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND22" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R3" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R4" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP15" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R6" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="Q2" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="GND23" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP17" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP18" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="C5" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND25" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="IC3" library="SamacSys_Parts" deviceset="VREG-SOT-223" device=""/>
<part name="CON3" library="SamacSys_Parts" deviceset="CON-89-3_81-PLUG" device=""/>
<part name="IC2" library="SamacSys_Parts" deviceset="NL27WZ04DTT1G" device=""/>
<part name="SUP11" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND18" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C6" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="D2" library="SamacSys_Parts" deviceset="TVS-DIODE-SMA" device=""/>
<part name="D4" library="SamacSys_Parts" deviceset="TVS-DIODE-SMA" device=""/>
<part name="R11" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R9" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R8" library="SamacSys_Parts" deviceset="RESISTOR-0805" device=""/>
<part name="R10" library="SamacSys_Parts" deviceset="RESISTOR-0805" device=""/>
<part name="GND19" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND26" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND27" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND28" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP12" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP13" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND29" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="Q4" library="SamacSys_Parts" deviceset="N-MOSFET" device=""/>
<part name="R14" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R12" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R15" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R13" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND30" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND31" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="Q3" library="SamacSys_Parts" deviceset="N-MOSFET" device=""/>
<part name="GND32" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND33" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R7" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND34" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND35" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND36" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND37" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="F1" library="SamacSys_Parts" deviceset="FUSE-0805" device=""/>
<part name="Q6" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="R23" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R22" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND38" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C8" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="C7" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND39" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND40" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C12" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="C13" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="SUP19" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND41" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND42" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND43" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="D6" library="SamacSys_Parts" deviceset="DIODE-DO-214AC-SMA" device=""/>
<part name="GND14" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="U$2" library="SamacSys_Parts" deviceset="VEXT" device=""/>
<part name="Q5" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="SUP16" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R2" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="D1" library="SamacSys_Parts" deviceset="ZENER-SOD123" device=""/>
<part name="SUP10" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="D5" library="SamacSys_Parts" deviceset="SCHOTTKY-SERIE-3P" device=""/>
<part name="D3" library="SamacSys_Parts" deviceset="SCHOTTKY-SERIE-3P" device=""/>
<part name="GND24" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="IC1" library="SamacSys_Parts" deviceset="S-5712-SOT23-3" device=""/>
<part name="CON2" library="SamacSys_Parts" deviceset="HEADER-8P-2R-DIP-2_0MM" device=""/>
<part name="U$1" library="SamacSys_Parts" deviceset="VEXT" device=""/>
<part name="ANT1" library="SamacSys_Parts" deviceset="SMA-RF" device=""/>
<part name="R25" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND11" library="SamacSys_Parts" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="MOD1" gate="G$1" x="-27.94" y="17.78" smashed="yes">
<attribute name="NAME" x="-21.59" y="26.416" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-21.59" y="28.194" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="MOD2" gate="G$1" x="86.36" y="10.16" smashed="yes">
<attribute name="NAME" x="92.71" y="21.336" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="92.71" y="23.114" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND1" gate="1" x="-27.94" y="20.32" smashed="yes" rot="R180">
<attribute name="VALUE" x="-25.4" y="23.368" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND2" gate="1" x="-55.88" y="22.86" smashed="yes" rot="R270">
<attribute name="VALUE" x="-58.928" y="25.4" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C2" gate="G$1" x="-53.34" y="17.78" smashed="yes">
<attribute name="NAME" x="-51.308" y="18.796" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-49.53" y="22.606" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND3" gate="1" x="-55.88" y="17.78" smashed="yes" rot="R270">
<attribute name="VALUE" x="-58.928" y="20.32" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R1" gate="G$1" x="-73.66" y="17.78" smashed="yes">
<attribute name="NAME" x="-68.58" y="19.812" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-70.612" y="21.59" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C3" gate="G$1" x="-73.66" y="22.86" smashed="yes">
<attribute name="NAME" x="-68.834" y="26.162" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-69.85" y="27.686" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP2" gate="G$1" x="-76.2" y="17.78" smashed="yes" rot="R90">
<attribute name="VALUE" x="-78.105" y="15.875" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND4" gate="1" x="-76.2" y="22.86" smashed="yes" rot="R270">
<attribute name="VALUE" x="-79.248" y="25.4" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND5" gate="1" x="83.82" y="10.16" smashed="yes" rot="R270">
<attribute name="VALUE" x="80.772" y="12.7" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND6" gate="1" x="101.6" y="-33.02" smashed="yes">
<attribute name="VALUE" x="99.06" y="-36.068" size="1.778" layer="96"/>
</instance>
<instance part="GND7" gate="1" x="132.08" y="-7.62" smashed="yes" rot="R90">
<attribute name="VALUE" x="135.128" y="-10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND8" gate="1" x="132.08" y="10.16" smashed="yes" rot="R90">
<attribute name="VALUE" x="135.128" y="7.62" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND9" gate="1" x="116.84" y="25.4" smashed="yes" rot="R90">
<attribute name="VALUE" x="119.888" y="22.86" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND10" gate="1" x="99.06" y="25.4" smashed="yes" rot="R270">
<attribute name="VALUE" x="96.012" y="27.94" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R21" gate="G$1" x="78.74" y="10.16" smashed="yes" rot="R270">
<attribute name="NAME" x="81.026" y="6.35" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="82.55" y="7.112" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="R19" gate="G$1" x="76.2" y="10.16" smashed="yes" rot="R270">
<attribute name="NAME" x="73.914" y="6.096" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="80.01" y="7.112" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="SUP3" gate="G$1" x="78.74" y="12.7" smashed="yes">
<attribute name="VALUE" x="76.835" y="14.605" size="1.778" layer="96"/>
</instance>
<instance part="SUP4" gate="G$1" x="76.2" y="12.7" smashed="yes">
<attribute name="VALUE" x="74.295" y="14.605" size="1.778" layer="96"/>
</instance>
<instance part="C1" gate="G$1" x="-53.34" y="22.86" smashed="yes">
<attribute name="NAME" x="-51.054" y="23.876" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-49.53" y="27.686" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C9" gate="G$1" x="86.36" y="-27.94" smashed="yes" rot="R90">
<attribute name="NAME" x="83.058" y="-23.114" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="81.534" y="-24.13" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND12" gate="1" x="86.36" y="-27.94" smashed="yes">
<attribute name="VALUE" x="83.82" y="-30.988" size="1.778" layer="96"/>
</instance>
<instance part="SUP5" gate="G$1" x="83.82" y="-15.24" smashed="yes" rot="R90">
<attribute name="VALUE" x="81.915" y="-17.145" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R24" gate="G$1" x="139.7" y="0" smashed="yes" rot="R90">
<attribute name="NAME" x="141.732" y="4.572" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="135.89" y="3.048" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C11" gate="G$1" x="139.7" y="-12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="136.398" y="-8.636" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="134.874" y="-8.89" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP6" gate="G$1" x="139.7" y="15.24" smashed="yes">
<attribute name="VALUE" x="137.795" y="17.145" size="1.778" layer="96"/>
</instance>
<instance part="GND13" gate="1" x="139.7" y="-15.24" smashed="yes">
<attribute name="VALUE" x="137.16" y="-18.288" size="1.778" layer="96"/>
</instance>
<instance part="GND15" gate="1" x="27.94" y="17.78" smashed="yes" rot="R90">
<attribute name="VALUE" x="30.988" y="15.24" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R18" gate="G$1" x="63.5" y="-7.62" smashed="yes">
<attribute name="NAME" x="67.31" y="-5.588" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="66.548" y="-3.81" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R20" gate="G$1" x="63.5" y="-10.16" smashed="yes">
<attribute name="NAME" x="67.31" y="-12.192" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="66.548" y="-6.35" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R17" gate="G$1" x="43.18" y="12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="45.212" y="16.51" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="39.37" y="15.748" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R16" gate="G$1" x="40.64" y="12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="38.608" y="16.51" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="36.83" y="15.748" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP7" gate="G$1" x="40.64" y="27.94" smashed="yes">
<attribute name="VALUE" x="38.735" y="29.845" size="1.778" layer="96"/>
</instance>
<instance part="SUP8" gate="G$1" x="43.18" y="27.94" smashed="yes">
<attribute name="VALUE" x="41.275" y="29.845" size="1.778" layer="96"/>
</instance>
<instance part="GND16" gate="1" x="48.26" y="15.24" smashed="yes" rot="R180">
<attribute name="VALUE" x="50.8" y="18.288" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUP9" gate="G$1" x="68.58" y="7.62" smashed="yes" rot="R270">
<attribute name="VALUE" x="70.485" y="9.525" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND17" gate="1" x="68.58" y="12.7" smashed="yes" rot="R90">
<attribute name="VALUE" x="71.628" y="10.16" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="CON1" gate="G$1" x="48.26" y="25.4" smashed="yes">
<attribute name="NAME" x="53.34" y="28.956" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="53.34" y="30.734" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND20" gate="1" x="48.26" y="20.32" smashed="yes">
<attribute name="VALUE" x="45.72" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="Q1" gate="G$1" x="-104.14" y="20.32" smashed="yes">
<attribute name="NAME" x="-102.87" y="27.178" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-98.552" y="22.098" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R5" gate="G$1" x="-88.9" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-90.932" y="19.812" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-92.71" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C4" gate="G$1" x="-83.82" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-84.836" y="17.272" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-88.646" y="19.05" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND21" gate="1" x="-88.9" y="12.7" smashed="yes">
<attribute name="VALUE" x="-91.44" y="9.652" size="1.778" layer="96"/>
</instance>
<instance part="GND22" gate="1" x="-83.82" y="12.7" smashed="yes">
<attribute name="VALUE" x="-86.36" y="9.652" size="1.778" layer="96"/>
</instance>
<instance part="R3" gate="G$1" x="-106.68" y="25.4" smashed="yes" rot="R180">
<attribute name="NAME" x="-111.252" y="23.368" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-109.728" y="21.59" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="R4" gate="G$1" x="-93.98" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-96.012" y="19.812" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-97.79" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP15" gate="G$1" x="-99.06" y="30.48" smashed="yes">
<attribute name="VALUE" x="-100.965" y="32.385" size="1.778" layer="96"/>
</instance>
<instance part="R6" gate="G$1" x="-127" y="-2.54" smashed="yes" rot="R90">
<attribute name="NAME" x="-129.032" y="1.524" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-130.81" y="0.508" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="Q2" gate="G$1" x="-129.54" y="5.08" smashed="yes" rot="R90">
<attribute name="NAME" x="-132.08" y="3.556" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-131.318" y="10.668" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND23" gate="1" x="-127" y="-5.08" smashed="yes">
<attribute name="VALUE" x="-129.54" y="-8.128" size="1.778" layer="96"/>
</instance>
<instance part="SUP17" gate="G$1" x="-139.7" y="10.16" smashed="yes" rot="R90">
<attribute name="VALUE" x="-141.605" y="8.255" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP18" gate="G$1" x="-147.32" y="7.62" smashed="yes">
<attribute name="VALUE" x="-149.225" y="9.525" size="1.778" layer="96"/>
</instance>
<instance part="C5" gate="G$1" x="-160.02" y="-12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="-163.322" y="-7.874" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-164.846" y="-8.89" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND25" gate="1" x="-160.02" y="-15.24" smashed="yes">
<attribute name="VALUE" x="-162.56" y="-18.288" size="1.778" layer="96"/>
</instance>
<instance part="IC3" gate="G$1" x="68.58" y="-35.56" smashed="yes">
<attribute name="NAME" x="50.8" y="-29.464" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="50.8" y="-27.686" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="CON3" gate="G$1" x="-78.74" y="-43.18" smashed="yes" rot="MR270">
<attribute name="NAME" x="-82.296" y="-48.26" size="1.778" layer="95" rot="MR270" align="center-left"/>
<attribute name="VALUE" x="-84.074" y="-48.26" size="1.778" layer="96" rot="MR270" align="center-left"/>
</instance>
<instance part="IC2" gate="G$1" x="-149.86" y="-35.56" smashed="yes">
<attribute name="NAME" x="-144.526" y="-32.004" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-144.526" y="-30.226" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP11" gate="G$1" x="-114.3" y="-38.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="-112.395" y="-36.195" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND18" gate="1" x="-116.84" y="-53.34" smashed="yes">
<attribute name="VALUE" x="-119.38" y="-56.388" size="1.778" layer="96"/>
</instance>
<instance part="C6" gate="G$1" x="-116.84" y="-38.1" smashed="yes" rot="R270">
<attribute name="NAME" x="-113.792" y="-42.926" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-112.014" y="-41.91" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="D2" gate="G$1" x="-101.6" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-103.632" y="-13.208" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-105.41" y="-16.256" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="D4" gate="G$1" x="-106.68" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-108.966" y="-13.208" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-110.49" y="-16.256" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R11" gate="G$1" x="-73.66" y="0" smashed="yes">
<attribute name="NAME" x="-69.596" y="2.032" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-70.612" y="3.81" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R9" gate="G$1" x="-73.66" y="-2.54" smashed="yes">
<attribute name="NAME" x="-69.088" y="-4.572" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-70.612" y="1.27" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R8" gate="G$1" x="-88.9" y="-2.54" smashed="yes" rot="R180">
<attribute name="NAME" x="-93.726" y="-4.572" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-91.948" y="-6.35" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="R10" gate="G$1" x="-88.9" y="0" smashed="yes" rot="R180">
<attribute name="NAME" x="-93.218" y="1.778" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-91.948" y="-3.81" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="GND19" gate="1" x="-88.9" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-91.44" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND26" gate="1" x="-78.74" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-81.28" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND27" gate="1" x="-101.6" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-104.14" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND28" gate="1" x="-106.68" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-109.22" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="SUP12" gate="G$1" x="-83.82" y="-20.32" smashed="yes" rot="R180">
<attribute name="VALUE" x="-81.915" y="-22.225" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUP13" gate="G$1" x="-73.66" y="-20.32" smashed="yes" rot="R180">
<attribute name="VALUE" x="-71.755" y="-22.225" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND29" gate="1" x="71.12" y="25.4" smashed="yes">
<attribute name="VALUE" x="68.58" y="22.352" size="1.778" layer="96"/>
</instance>
<instance part="Q4" gate="G$1" x="-38.1" y="-43.18" smashed="yes">
<attribute name="NAME" x="-32.512" y="-39.624" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-32.512" y="-41.402" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R14" gate="G$1" x="-40.64" y="-20.32" smashed="yes" rot="R90">
<attribute name="NAME" x="-38.1" y="-16.256" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-44.45" y="-17.272" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R12" gate="G$1" x="-43.18" y="-20.32" smashed="yes" rot="R90">
<attribute name="NAME" x="-45.212" y="-16.51" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-46.99" y="-17.272" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R15" gate="G$1" x="-40.64" y="-50.8" smashed="yes" rot="R90">
<attribute name="NAME" x="-42.672" y="-46.99" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-44.45" y="-47.752" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R13" gate="G$1" x="-53.34" y="-50.8" smashed="yes" rot="R90">
<attribute name="NAME" x="-55.626" y="-46.736" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-57.15" y="-47.752" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND30" gate="1" x="-40.64" y="-53.34" smashed="yes">
<attribute name="VALUE" x="-43.18" y="-56.388" size="1.778" layer="96"/>
</instance>
<instance part="GND31" gate="1" x="-33.02" y="-53.34" smashed="yes">
<attribute name="VALUE" x="-35.56" y="-56.388" size="1.778" layer="96"/>
</instance>
<instance part="Q3" gate="G$1" x="-50.8" y="-43.18" smashed="yes">
<attribute name="NAME" x="-45.212" y="-39.624" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-45.212" y="-41.402" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND32" gate="1" x="-53.34" y="-53.34" smashed="yes">
<attribute name="VALUE" x="-55.88" y="-56.388" size="1.778" layer="96"/>
</instance>
<instance part="GND33" gate="1" x="-45.72" y="-53.34" smashed="yes">
<attribute name="VALUE" x="-48.26" y="-56.388" size="1.778" layer="96"/>
</instance>
<instance part="R7" gate="G$1" x="-134.62" y="-7.62" smashed="yes" rot="R90">
<attribute name="NAME" x="-136.652" y="-2.54" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-138.43" y="-4.572" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND34" gate="1" x="-152.4" y="-38.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="-155.448" y="-35.56" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND35" gate="1" x="-76.2" y="-40.64" smashed="yes" rot="R180">
<attribute name="VALUE" x="-73.66" y="-37.592" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND36" gate="1" x="-71.12" y="-40.64" smashed="yes" rot="R180">
<attribute name="VALUE" x="-68.58" y="-37.592" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND37" gate="1" x="-55.88" y="27.94" smashed="yes" rot="R270">
<attribute name="VALUE" x="-58.928" y="30.48" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="F1" gate="G$1" x="50.8" y="33.02" smashed="yes">
<attribute name="NAME" x="53.34" y="35.052" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="53.34" y="36.83" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="Q6" gate="G$1" x="81.28" y="27.94" smashed="yes" rot="R90">
<attribute name="NAME" x="78.994" y="33.528" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="79.502" y="33.528" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R23" gate="G$1" x="76.2" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="78.486" y="19.05" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="72.39" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R22" gate="G$1" x="73.66" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="71.628" y="18.796" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="69.85" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND38" gate="1" x="81.28" y="15.24" smashed="yes" rot="R90">
<attribute name="VALUE" x="84.328" y="12.7" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C8" gate="G$1" x="-53.34" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-54.356" y="-15.494" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-58.166" y="-13.97" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C7" gate="G$1" x="-58.42" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="-59.436" y="-15.494" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-63.246" y="-13.97" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND39" gate="1" x="-58.42" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-60.96" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND40" gate="1" x="-53.34" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-55.88" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="C12" gate="G$1" x="45.72" y="-48.26" smashed="yes" rot="R90">
<attribute name="NAME" x="42.672" y="-44.45" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="40.894" y="-44.45" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C13" gate="G$1" x="71.12" y="-48.26" smashed="yes" rot="R90">
<attribute name="NAME" x="68.072" y="-44.45" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="66.294" y="-44.45" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP19" gate="G$1" x="71.12" y="-33.02" smashed="yes">
<attribute name="VALUE" x="69.215" y="-31.115" size="1.778" layer="96"/>
</instance>
<instance part="GND41" gate="1" x="45.72" y="-50.8" smashed="yes">
<attribute name="VALUE" x="43.18" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND42" gate="1" x="58.42" y="-50.8" smashed="yes">
<attribute name="VALUE" x="55.88" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND43" gate="1" x="71.12" y="-50.8" smashed="yes">
<attribute name="VALUE" x="68.58" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="D6" gate="G$1" x="45.72" y="-35.56" smashed="yes" rot="R180">
<attribute name="NAME" x="40.894" y="-33.274" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="44.196" y="-39.37" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="GND14" gate="1" x="-12.7" y="-33.02" smashed="yes">
<attribute name="VALUE" x="-15.24" y="-36.068" size="1.778" layer="96"/>
</instance>
<instance part="U$2" gate="G$1" x="-78.74" y="-38.1" smashed="yes">
<attribute name="VALUE" x="-80.645" y="-36.195" size="1.778" layer="96"/>
</instance>
<instance part="Q5" gate="G$1" x="66.04" y="27.94" smashed="yes" rot="MR90">
<attribute name="NAME" x="67.818" y="33.528" size="1.778" layer="95" rot="MR90" align="center-left"/>
<attribute name="VALUE" x="67.818" y="33.528" size="1.778" layer="96" rot="MR90" align="center-left"/>
</instance>
<instance part="SUP16" gate="G$1" x="-121.92" y="20.32" smashed="yes" rot="R90">
<attribute name="VALUE" x="-123.825" y="18.415" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R2" gate="G$1" x="-106.68" y="20.32" smashed="yes" rot="R180">
<attribute name="NAME" x="-111.252" y="18.288" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-109.728" y="16.51" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="D1" gate="G$1" x="-40.64" y="27.94" smashed="yes" rot="R180">
<attribute name="NAME" x="-45.72" y="30.226" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-42.164" y="24.13" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="SUP10" gate="G$1" x="86.36" y="33.02" smashed="yes" rot="R270">
<attribute name="VALUE" x="88.265" y="34.925" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP1" gate="G$1" x="-40.64" y="33.02" smashed="yes">
<attribute name="VALUE" x="-42.545" y="34.925" size="1.778" layer="96"/>
</instance>
<instance part="D5" gate="G$1" x="-86.36" y="-5.08" smashed="yes">
<attribute name="VALUE" x="-89.916" y="-1.27" size="1.778" layer="96" align="center-left"/>
<attribute name="NAME" x="-87.884" y="-9.398" size="1.778" layer="95"/>
</instance>
<instance part="D3" gate="G$1" x="-76.2" y="-5.08" smashed="yes">
<attribute name="VALUE" x="-79.756" y="-1.27" size="1.778" layer="96" align="center-left"/>
<attribute name="NAME" x="-77.724" y="-9.398" size="1.778" layer="95"/>
</instance>
<instance part="GND24" gate="1" x="-147.32" y="-22.86" smashed="yes">
<attribute name="VALUE" x="-149.86" y="-25.908" size="1.778" layer="96"/>
</instance>
<instance part="IC1" gate="G$1" x="-137.16" y="-7.62" smashed="yes">
<attribute name="NAME" x="-154.94" y="1.016" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-154.94" y="2.794" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="CON2" gate="G$1" x="48.26" y="12.7" smashed="yes">
<attribute name="NAME" x="53.34" y="16.256" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="53.34" y="18.034" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="U$1" gate="G$1" x="33.02" y="-33.02" smashed="yes">
<attribute name="VALUE" x="31.115" y="-31.115" size="1.778" layer="96"/>
</instance>
<instance part="ANT1" gate="G$1" x="104.14" y="33.02" smashed="yes">
<attribute name="VALUE" x="102.235" y="34.925" size="1.778" layer="96"/>
</instance>
<instance part="R25" gate="G$1" x="137.16" y="0" smashed="yes" rot="R90">
<attribute name="NAME" x="135.128" y="4.572" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="133.35" y="3.048" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND11" gate="1" x="106.68" y="27.94" smashed="yes">
<attribute name="VALUE" x="104.14" y="24.892" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="GND1" gate="1" pin="GND"/>
<pinref part="MOD1" gate="G$1" pin="GND_1"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="1"/>
<pinref part="GND3" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C3" gate="G$1" pin="1"/>
<pinref part="GND4" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_1"/>
<pinref part="GND5" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_2"/>
<pinref part="GND6" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_3"/>
<pinref part="GND7" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_4"/>
<pinref part="GND8" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_5"/>
<pinref part="GND9" gate="1" pin="GND"/>
<pinref part="MOD2" gate="G$1" pin="GND_8"/>
<wire x1="106.68" y1="25.4" x2="109.22" y2="25.4" width="0.1524" layer="91"/>
<junction x="114.3" y="25.4"/>
<pinref part="MOD2" gate="G$1" pin="GND_6"/>
<wire x1="109.22" y1="25.4" x2="111.76" y2="25.4" width="0.1524" layer="91"/>
<wire x1="111.76" y1="25.4" x2="114.3" y2="25.4" width="0.1524" layer="91"/>
<junction x="111.76" y="25.4"/>
<pinref part="MOD2" gate="G$1" pin="GND_7"/>
<junction x="109.22" y="25.4"/>
</segment>
<segment>
<pinref part="MOD2" gate="G$1" pin="GND_9"/>
<pinref part="GND10" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND2" gate="1" pin="GND"/>
<pinref part="C1" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="C9" gate="G$1" pin="1"/>
<pinref part="GND12" gate="1" pin="GND"/>
<wire x1="86.36" y1="-27.94" x2="86.36" y2="-25.4" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C11" gate="G$1" pin="1"/>
<pinref part="GND13" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_3"/>
<pinref part="GND15" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="2"/>
<pinref part="GND20" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<pinref part="GND21" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C4" gate="G$1" pin="1"/>
<pinref part="GND22" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<pinref part="GND23" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C5" gate="G$1" pin="1"/>
<pinref part="GND25" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C6" gate="G$1" pin="2"/>
<pinref part="GND18" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="D2" gate="G$1" pin="1"/>
<pinref part="GND27" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="D4" gate="G$1" pin="1"/>
<pinref part="GND28" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R15" gate="G$1" pin="1"/>
<pinref part="GND30" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="Q4" gate="G$1" pin="S"/>
<pinref part="GND31" gate="1" pin="GND"/>
<wire x1="-33.02" y1="-45.72" x2="-33.02" y2="-50.8" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R13" gate="G$1" pin="1"/>
<pinref part="GND32" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="Q3" gate="G$1" pin="S"/>
<pinref part="GND33" gate="1" pin="GND"/>
<wire x1="-45.72" y1="-45.72" x2="-45.72" y2="-50.8" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="IC2" gate="G$1" pin="GND"/>
<pinref part="GND34" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON3" gate="G$1" pin="2"/>
<pinref part="GND35" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON3" gate="G$1" pin="4"/>
<pinref part="GND36" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R23" gate="G$1" pin="1"/>
<pinref part="GND38" gate="1" pin="GND"/>
<wire x1="76.2" y1="15.24" x2="78.74" y2="15.24" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C7" gate="G$1" pin="1"/>
<pinref part="GND39" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C8" gate="G$1" pin="1"/>
<pinref part="GND40" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C12" gate="G$1" pin="1"/>
<pinref part="GND41" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="GND"/>
<pinref part="GND42" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C13" gate="G$1" pin="1"/>
<pinref part="GND43" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND14" gate="1" pin="GND"/>
<pinref part="MOD1" gate="G$1" pin="GND_2"/>
</segment>
<segment>
<pinref part="Q5" gate="G$1" pin="G"/>
<pinref part="GND29" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="D1" gate="G$1" pin="A"/>
<pinref part="GND37" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND19" gate="1" pin="GND"/>
<pinref part="D5" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="GND26" gate="1" pin="GND"/>
<pinref part="D3" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VSS"/>
<pinref part="GND24" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND16" gate="1" pin="GND"/>
<pinref part="CON2" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="GND17" gate="1" pin="GND"/>
<pinref part="CON2" gate="G$1" pin="5"/>
</segment>
<segment>
<pinref part="ANT1" gate="G$1" pin="GND"/>
<pinref part="GND11" gate="1" pin="GND"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R21" gate="G$1" pin="1"/>
<pinref part="SUP3" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R19" gate="G$1" pin="1"/>
<pinref part="SUP4" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="C9" gate="G$1" pin="2"/>
<pinref part="SUP5" gate="G$1" pin="VCC"/>
<pinref part="MOD2" gate="G$1" pin="VCC"/>
<junction x="86.36" y="-15.24"/>
</segment>
<segment>
<pinref part="R24" gate="G$1" pin="2"/>
<pinref part="SUP6" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R16" gate="G$1" pin="2"/>
<pinref part="SUP7" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R17" gate="G$1" pin="2"/>
<pinref part="SUP8" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="Q1" gate="G$1" pin="S"/>
<pinref part="SUP15" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC2" gate="G$1" pin="VCC"/>
<pinref part="SUP11" gate="G$1" pin="VCC"/>
<wire x1="-116.84" y1="-38.1" x2="-121.92" y2="-38.1" width="0.1524" layer="91"/>
<pinref part="C6" gate="G$1" pin="1"/>
<junction x="-116.84" y="-38.1"/>
</segment>
<segment>
<pinref part="IC3" gate="G$1" pin="OUT"/>
<pinref part="C13" gate="G$1" pin="2"/>
<pinref part="SUP19" gate="G$1" pin="VCC"/>
<junction x="71.12" y="-35.56"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="SUP16" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="Q6" gate="G$1" pin="D"/>
<pinref part="SUP10" gate="G$1" pin="VCC"/>
</segment>
<segment>
<wire x1="-40.64" y1="17.78" x2="-40.64" y2="22.86" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="2"/>
<pinref part="MOD1" gate="G$1" pin="3V3"/>
<wire x1="-40.64" y1="17.78" x2="-40.64" y2="15.24" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="15.24" x2="-27.94" y2="15.24" width="0.1524" layer="91"/>
<junction x="-40.64" y="17.78"/>
<pinref part="C1" gate="G$1" pin="2"/>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
<wire x1="-40.64" y1="30.48" x2="-40.64" y2="27.94" width="0.1524" layer="91"/>
<junction x="-40.64" y="22.86"/>
<pinref part="D1" gate="G$1" pin="K"/>
<wire x1="-40.64" y1="27.94" x2="-40.64" y2="22.86" width="0.1524" layer="91"/>
<junction x="-40.64" y="27.94"/>
</segment>
<segment>
<pinref part="SUP12" gate="G$1" pin="VCC"/>
<pinref part="D5" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="SUP13" gate="G$1" pin="VCC"/>
<pinref part="D3" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="Q2" gate="G$1" pin="S"/>
<pinref part="SUP17" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="SUP18" gate="G$1" pin="VCC"/>
<pinref part="IC1" gate="G$1" pin="VDD"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="-160.02" y1="0" x2="-160.02" y2="5.08" width="0.1524" layer="91"/>
<wire x1="-160.02" y1="5.08" x2="-147.32" y2="5.08" width="0.1524" layer="91"/>
<junction x="-147.32" y="5.08"/>
</segment>
<segment>
<pinref part="SUP9" gate="G$1" pin="VCC"/>
<pinref part="CON2" gate="G$1" pin="7"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="2"/>
<pinref part="C3" gate="G$1" pin="2"/>
<wire x1="-60.96" y1="17.78" x2="-60.96" y2="22.86" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="EN"/>
<wire x1="-60.96" y1="17.78" x2="-60.96" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="12.7" x2="-27.94" y2="12.7" width="0.1524" layer="91"/>
<junction x="-60.96" y="17.78"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<wire x1="78.74" y1="-10.16" x2="78.74" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="MOD2" gate="G$1" pin="PA9/TX1"/>
<wire x1="78.74" y1="-2.54" x2="86.36" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="R21" gate="G$1" pin="2"/>
<junction x="78.74" y="-2.54"/>
<pinref part="R20" gate="G$1" pin="2"/>
<wire x1="78.74" y1="-10.16" x2="76.2" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<wire x1="76.2" y1="-7.62" x2="76.2" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="MOD2" gate="G$1" pin="PA10/RX1"/>
<wire x1="76.2" y1="-5.08" x2="86.36" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="R19" gate="G$1" pin="2"/>
<wire x1="76.2" y1="-5.08" x2="76.2" y2="-2.54" width="0.1524" layer="91"/>
<junction x="76.2" y="-5.08"/>
<pinref part="R18" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO23"/>
<wire x1="25.4" y1="15.24" x2="35.56" y2="15.24" width="0.1524" layer="91"/>
<wire x1="35.56" y1="15.24" x2="35.56" y2="38.1" width="0.1524" layer="91"/>
<wire x1="35.56" y1="38.1" x2="137.16" y2="38.1" width="0.1524" layer="91"/>
<pinref part="R25" gate="G$1" pin="2"/>
<wire x1="137.16" y1="38.1" x2="137.16" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="R18" gate="G$1" pin="1"/>
<pinref part="MOD1" gate="G$1" pin="IO17/TXD2"/>
<wire x1="63.5" y1="-7.62" x2="25.4" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="R20" gate="G$1" pin="1"/>
<pinref part="MOD1" gate="G$1" pin="IO16/RXD2"/>
<wire x1="63.5" y1="-10.16" x2="25.4" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="TXD0"/>
<wire x1="25.4" y1="10.16" x2="40.64" y2="10.16" width="0.1524" layer="91"/>
<pinref part="R16" gate="G$1" pin="1"/>
<wire x1="40.64" y1="10.16" x2="48.26" y2="10.16" width="0.1524" layer="91"/>
<wire x1="40.64" y1="12.7" x2="40.64" y2="10.16" width="0.1524" layer="91"/>
<junction x="40.64" y="10.16"/>
<pinref part="CON2" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RXD0"/>
<wire x1="25.4" y1="7.62" x2="43.18" y2="7.62" width="0.1524" layer="91"/>
<pinref part="R17" gate="G$1" pin="1"/>
<wire x1="43.18" y1="7.62" x2="48.26" y2="7.62" width="0.1524" layer="91"/>
<wire x1="43.18" y1="12.7" x2="43.18" y2="7.62" width="0.1524" layer="91"/>
<junction x="43.18" y="7.62"/>
<pinref part="CON2" gate="G$1" pin="3"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<wire x1="66.04" y1="10.16" x2="71.12" y2="10.16" width="0.1524" layer="91"/>
<wire x1="71.12" y1="10.16" x2="71.12" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="71.12" y1="-2.54" x2="43.18" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="43.18" y1="-2.54" x2="43.18" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="IO0"/>
<wire x1="43.18" y1="-15.24" x2="25.4" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="6"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="Q1" gate="G$1" pin="D"/>
<wire x1="-99.06" y1="17.78" x2="-99.06" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-99.06" y1="12.7" x2="-93.98" y2="12.7" width="0.1524" layer="91"/>
<pinref part="R4" gate="G$1" pin="1"/>
<wire x1="-93.98" y1="12.7" x2="-93.98" y2="15.24" width="0.1524" layer="91"/>
<pinref part="CON3" gate="G$1" pin="3"/>
<wire x1="-73.66" y1="-43.18" x2="-73.66" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-73.66" y1="-33.02" x2="-111.76" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-111.76" y1="-33.02" x2="-111.76" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-111.76" y1="12.7" x2="-99.06" y2="12.7" width="0.1524" layer="91"/>
<junction x="-99.06" y="12.7"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="R6" gate="G$1" pin="2"/>
<pinref part="Q2" gate="G$1" pin="D"/>
<pinref part="MOD1" gate="G$1" pin="IO35/SENSOR_VN"/>
<wire x1="-27.94" y1="7.62" x2="-121.92" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-121.92" y1="7.62" x2="-121.92" y2="10.16" width="0.1524" layer="91"/>
<wire x1="-121.92" y1="10.16" x2="-127" y2="10.16" width="0.1524" layer="91"/>
<junction x="-127" y="10.16"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO32"/>
<pinref part="R11" gate="G$1" pin="2"/>
<wire x1="-60.96" y1="0" x2="-58.42" y2="0" width="0.1524" layer="91"/>
<pinref part="C7" gate="G$1" pin="2"/>
<wire x1="-58.42" y1="0" x2="-48.26" y2="0" width="0.1524" layer="91"/>
<wire x1="-48.26" y1="0" x2="-30.48" y2="0" width="0.1524" layer="91"/>
<wire x1="-30.48" y1="0" x2="-27.94" y2="0" width="0.1524" layer="91"/>
<wire x1="-58.42" y1="-5.08" x2="-58.42" y2="0" width="0.1524" layer="91"/>
<junction x="-58.42" y="0"/>
<wire x1="-48.26" y1="-22.86" x2="-48.26" y2="0" width="0.1524" layer="91"/>
<junction x="-48.26" y="0"/>
<wire x1="-48.26" y1="-22.86" x2="-127" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="-127" y1="-22.86" x2="-127" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-127" y1="-27.94" x2="-149.86" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="A1"/>
<wire x1="-149.86" y1="-27.94" x2="-149.86" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="-30.48" y1="0" x2="-30.48" y2="30.48" width="0.1524" layer="91"/>
<junction x="-30.48" y="0"/>
<wire x1="-30.48" y1="30.48" x2="30.48" y2="30.48" width="0.1524" layer="91"/>
<wire x1="30.48" y1="30.48" x2="30.48" y2="12.7" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="IO22/SCL"/>
<wire x1="30.48" y1="12.7" x2="25.4" y2="12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO33"/>
<pinref part="R9" gate="G$1" pin="2"/>
<wire x1="-60.96" y1="-2.54" x2="-53.34" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="C8" gate="G$1" pin="2"/>
<wire x1="-53.34" y1="-2.54" x2="-45.72" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-45.72" y1="-2.54" x2="-33.02" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-2.54" x2="-27.94" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-53.34" y1="-5.08" x2="-53.34" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-53.34" y="-2.54"/>
<wire x1="-45.72" y1="-25.4" x2="-45.72" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-45.72" y="-2.54"/>
<wire x1="-45.72" y1="-25.4" x2="-124.46" y2="-25.4" width="0.1524" layer="91"/>
<wire x1="-124.46" y1="-25.4" x2="-124.46" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="-124.46" y1="-30.48" x2="-154.94" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="A2"/>
<wire x1="-154.94" y1="-30.48" x2="-154.94" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-154.94" y1="-40.64" x2="-149.86" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-2.54" x2="-33.02" y2="33.02" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="33.02" x2="33.02" y2="33.02" width="0.1524" layer="91"/>
<junction x="-33.02" y="-2.54"/>
<pinref part="MOD1" gate="G$1" pin="IO21/SDA"/>
<wire x1="33.02" y1="33.02" x2="33.02" y2="5.08" width="0.1524" layer="91"/>
<wire x1="33.02" y1="5.08" x2="25.4" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO26"/>
<pinref part="R14" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="-7.62" x2="-40.64" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$27" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO25"/>
<pinref part="R12" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="-5.08" x2="-43.18" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="-5.08" x2="-43.18" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="R15" gate="G$1" pin="2"/>
<pinref part="Q4" gate="G$1" pin="G"/>
<wire x1="-40.64" y1="-38.1" x2="-38.1" y2="-38.1" width="0.1524" layer="91"/>
<pinref part="R14" gate="G$1" pin="1"/>
<wire x1="-40.64" y1="-20.32" x2="-40.64" y2="-38.1" width="0.1524" layer="91"/>
<junction x="-40.64" y="-38.1"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="R10" gate="G$1" pin="1"/>
<pinref part="R11" gate="G$1" pin="1"/>
<wire x1="-88.9" y1="0" x2="-86.36" y2="0" width="0.1524" layer="91"/>
<wire x1="-86.36" y1="0" x2="-73.66" y2="0" width="0.1524" layer="91"/>
<wire x1="-86.36" y1="-5.08" x2="-86.36" y2="0" width="0.1524" layer="91"/>
<junction x="-86.36" y="0"/>
<pinref part="D5" gate="G$1" pin="3"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="R8" gate="G$1" pin="1"/>
<pinref part="R9" gate="G$1" pin="1"/>
<wire x1="-88.9" y1="-2.54" x2="-76.2" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-76.2" y1="-2.54" x2="-73.66" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-76.2" y1="-5.08" x2="-76.2" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-76.2" y="-2.54"/>
<pinref part="D3" gate="G$1" pin="3"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="Q2" gate="G$1" pin="G"/>
<pinref part="R7" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="Q3" gate="G$1" pin="D"/>
<wire x1="-45.72" y1="-35.56" x2="-45.72" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="CON3" gate="G$1" pin="7"/>
<wire x1="-45.72" y1="-30.48" x2="-63.5" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="-63.5" y1="-30.48" x2="-63.5" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="CON1" gate="G$1" pin="1"/>
<pinref part="F1" gate="G$1" pin="1"/>
<wire x1="48.26" y1="25.4" x2="48.26" y2="33.02" width="0.1524" layer="91"/>
<wire x1="48.26" y1="33.02" x2="50.8" y2="33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$37" class="0">
<segment>
<pinref part="Q6" gate="G$1" pin="G"/>
<pinref part="R23" gate="G$1" pin="2"/>
<pinref part="R22" gate="G$1" pin="2"/>
<wire x1="73.66" y1="27.94" x2="76.2" y2="27.94" width="0.1524" layer="91"/>
<junction x="76.2" y="27.94"/>
</segment>
</net>
<net name="N$39" class="0">
<segment>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="-101.6" y1="-2.54" x2="-114.3" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="D2" gate="G$1" pin="2"/>
<wire x1="-101.6" y1="-5.08" x2="-101.6" y2="-2.54" width="0.1524" layer="91"/>
<junction x="-101.6" y="-2.54"/>
<wire x1="-114.3" y1="-2.54" x2="-114.3" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-114.3" y1="-27.94" x2="-66.04" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="CON3" gate="G$1" pin="6"/>
<wire x1="-66.04" y1="-27.94" x2="-66.04" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$40" class="0">
<segment>
<wire x1="-106.68" y1="0" x2="-116.84" y2="0" width="0.1524" layer="91"/>
<pinref part="D4" gate="G$1" pin="2"/>
<wire x1="-106.68" y1="-5.08" x2="-106.68" y2="0" width="0.1524" layer="91"/>
<junction x="-106.68" y="0"/>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="-101.6" y1="0" x2="-106.68" y2="0" width="0.1524" layer="91"/>
<wire x1="-116.84" y1="0" x2="-116.84" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="-116.84" y1="-30.48" x2="-68.58" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="CON3" gate="G$1" pin="5"/>
<wire x1="-68.58" y1="-30.48" x2="-68.58" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<pinref part="Q3" gate="G$1" pin="G"/>
<pinref part="R13" gate="G$1" pin="2"/>
<wire x1="-53.34" y1="-38.1" x2="-50.8" y2="-38.1" width="0.1524" layer="91"/>
<pinref part="R12" gate="G$1" pin="1"/>
<wire x1="-43.18" y1="-20.32" x2="-43.18" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="-27.94" x2="-53.34" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-53.34" y1="-27.94" x2="-53.34" y2="-38.1" width="0.1524" layer="91"/>
<junction x="-53.34" y="-38.1"/>
</segment>
</net>
<net name="VEXT" class="0">
<segment>
<pinref part="U$2" gate="G$1" pin="VEXT"/>
<pinref part="CON3" gate="G$1" pin="1"/>
<wire x1="-78.74" y1="-40.64" x2="-78.74" y2="-43.18" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="D6" gate="G$1" pin="A"/>
<pinref part="U$1" gate="G$1" pin="VEXT"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO35"/>
<wire x1="-27.94" y1="2.54" x2="-119.38" y2="2.54" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="-Y2"/>
<wire x1="-119.38" y1="2.54" x2="-119.38" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-119.38" y1="-40.64" x2="-121.92" y2="-40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO34"/>
<wire x1="-27.94" y1="5.08" x2="-121.92" y2="5.08" width="0.1524" layer="91"/>
<pinref part="IC2" gate="G$1" pin="-Y1"/>
<wire x1="-121.92" y1="5.08" x2="-121.92" y2="-35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$30" class="0">
<segment>
<pinref part="Q4" gate="G$1" pin="D"/>
<wire x1="-33.02" y1="-35.56" x2="-33.02" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="CON3" gate="G$1" pin="8"/>
<wire x1="-33.02" y1="-33.02" x2="-60.96" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="-33.02" x2="-60.96" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="IO27"/>
<wire x1="-27.94" y1="-10.16" x2="-35.56" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-35.56" y1="-10.16" x2="-35.56" y2="38.1" width="0.1524" layer="91"/>
<wire x1="-35.56" y1="38.1" x2="-119.38" y2="38.1" width="0.1524" layer="91"/>
<pinref part="R3" gate="G$1" pin="2"/>
<wire x1="-119.38" y1="38.1" x2="-119.38" y2="25.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="Q1" gate="G$1" pin="G"/>
<pinref part="R3" gate="G$1" pin="1"/>
<wire x1="-106.68" y1="25.4" x2="-104.14" y2="25.4" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="-106.68" y1="25.4" x2="-106.68" y2="20.32" width="0.1524" layer="91"/>
<junction x="-106.68" y="25.4"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="F1" gate="G$1" pin="2"/>
<pinref part="Q5" gate="G$1" pin="D"/>
</segment>
</net>
<net name="N$31" class="0">
<segment>
<pinref part="Q6" gate="G$1" pin="S"/>
<pinref part="Q5" gate="G$1" pin="S"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="R7" gate="G$1" pin="1"/>
<pinref part="IC1" gate="G$1" pin="OUT"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="CON2" gate="G$1" pin="4"/>
<wire x1="48.26" y1="5.08" x2="48.26" y2="0" width="0.1524" layer="91"/>
<wire x1="48.26" y1="0" x2="66.04" y2="0" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="8"/>
<wire x1="66.04" y1="0" x2="66.04" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<pinref part="D6" gate="G$1" pin="K"/>
<pinref part="IC3" gate="G$1" pin="IN"/>
<pinref part="C12" gate="G$1" pin="2"/>
<wire x1="45.72" y1="-35.56" x2="45.72" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="45.72" y1="-5.08" x2="73.66" y2="-5.08" width="0.1524" layer="91"/>
<junction x="45.72" y="-35.56"/>
<pinref part="R22" gate="G$1" pin="1"/>
<wire x1="73.66" y1="-5.08" x2="73.66" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="MOD2" gate="G$1" pin="RF_OUT"/>
<pinref part="ANT1" gate="G$1" pin="ANT"/>
<wire x1="104.14" y1="30.48" x2="104.14" y2="25.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$38" class="0">
<segment>
<pinref part="C4" gate="G$1" pin="2"/>
<pinref part="R5" gate="G$1" pin="2"/>
<pinref part="R4" gate="G$1" pin="2"/>
<wire x1="-93.98" y1="27.94" x2="-88.9" y2="27.94" width="0.1524" layer="91"/>
<junction x="-88.9" y="27.94"/>
<wire x1="-88.9" y1="27.94" x2="-83.82" y2="27.94" width="0.1524" layer="91"/>
<wire x1="-83.82" y1="27.94" x2="-78.74" y2="27.94" width="0.1524" layer="91"/>
<junction x="-83.82" y="27.94"/>
<pinref part="MOD1" gate="G$1" pin="IO36/SENSOR_VP"/>
<wire x1="-78.74" y1="27.94" x2="-78.74" y2="10.16" width="0.1524" layer="91"/>
<wire x1="-78.74" y1="10.16" x2="-27.94" y2="10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="R24" gate="G$1" pin="1"/>
<pinref part="C11" gate="G$1" pin="2"/>
<junction x="139.7" y="0"/>
<pinref part="MOD2" gate="G$1" pin="RST"/>
<wire x1="129.54" y1="0" x2="137.16" y2="0" width="0.1524" layer="91"/>
<pinref part="R25" gate="G$1" pin="1"/>
<wire x1="137.16" y1="0" x2="139.7" y2="0" width="0.1524" layer="91"/>
<junction x="137.16" y="0"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
