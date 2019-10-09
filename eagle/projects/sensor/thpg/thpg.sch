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
<package name="QFN-44">
<description>&lt;b&gt;ATMEGA32U4-MU&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-3.4" y="2.5" dx="1" dy="0.3" layer="1"/>
<smd name="2" x="-3.4" y="2" dx="1" dy="0.3" layer="1"/>
<smd name="3" x="-3.4" y="1.5" dx="1" dy="0.3" layer="1"/>
<smd name="4" x="-3.4" y="1" dx="1" dy="0.3" layer="1"/>
<smd name="5" x="-3.4" y="0.5" dx="1" dy="0.3" layer="1"/>
<smd name="6" x="-3.4" y="0" dx="1" dy="0.3" layer="1"/>
<smd name="7" x="-3.4" y="-0.5" dx="1" dy="0.3" layer="1"/>
<smd name="8" x="-3.4" y="-1" dx="1" dy="0.3" layer="1"/>
<smd name="9" x="-3.4" y="-1.5" dx="1" dy="0.3" layer="1"/>
<smd name="10" x="-3.4" y="-2" dx="1" dy="0.3" layer="1"/>
<smd name="11" x="-3.4" y="-2.5" dx="1" dy="0.3" layer="1"/>
<smd name="12" x="-2.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="13" x="-2" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="14" x="-1.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="15" x="-1" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="16" x="-0.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="17" x="0" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="18" x="0.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="19" x="1" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="20" x="1.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="21" x="2" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="22" x="2.5" y="-3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="23" x="3.4" y="-2.5" dx="1" dy="0.3" layer="1"/>
<smd name="24" x="3.4" y="-2" dx="1" dy="0.3" layer="1"/>
<smd name="25" x="3.4" y="-1.5" dx="1" dy="0.3" layer="1"/>
<smd name="26" x="3.4" y="-1" dx="1" dy="0.3" layer="1"/>
<smd name="27" x="3.4" y="-0.5" dx="1" dy="0.3" layer="1"/>
<smd name="28" x="3.4" y="0" dx="1" dy="0.3" layer="1"/>
<smd name="29" x="3.4" y="0.5" dx="1" dy="0.3" layer="1"/>
<smd name="30" x="3.4" y="1" dx="1" dy="0.3" layer="1"/>
<smd name="31" x="3.4" y="1.5" dx="1" dy="0.3" layer="1"/>
<smd name="32" x="3.4" y="2" dx="1" dy="0.3" layer="1"/>
<smd name="33" x="3.4" y="2.5" dx="1" dy="0.3" layer="1"/>
<smd name="34" x="2.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="35" x="2" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="36" x="1.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="37" x="1" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="38" x="0.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="39" x="0" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="40" x="-0.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="41" x="-1" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="42" x="-1.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="43" x="-2" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="44" x="-2.5" y="3.4" dx="1" dy="0.3" layer="1" rot="R90"/>
<smd name="45" x="0" y="0" dx="5.2" dy="5.2" layer="1" rot="R90"/>
<text x="0" y="1.05" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="1.05" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-3.5" y1="3.5" x2="-2.95" y2="3.5" width="0.2" layer="21"/>
<wire x1="2.9" y1="3.5" x2="3.5" y2="3.5" width="0.2" layer="21"/>
<wire x1="-3.5" y1="3.5" x2="-2.95" y2="3.5" width="0.1" layer="51"/>
<wire x1="2.9" y1="3.5" x2="3.5" y2="3.5" width="0.1" layer="51"/>
<wire x1="-3.5" y1="-3.5" x2="-2.9" y2="-3.5" width="0.1" layer="51"/>
<wire x1="2.9" y1="-3.5" x2="3.5" y2="-3.5" width="0.1" layer="51"/>
</package>
<package name="RAK811">
<smd name="1" x="-6.75" y="9" dx="2" dy="1.2" layer="1"/>
<smd name="2" x="-6.75" y="7.2" dx="2" dy="1.2" layer="1"/>
<smd name="3" x="-6.75" y="5.4" dx="2" dy="1.2" layer="1"/>
<smd name="4" x="-6.75" y="3.6" dx="2" dy="1.2" layer="1"/>
<smd name="5" x="-6.75" y="1.8" dx="2" dy="1.2" layer="1"/>
<smd name="6" x="-6.75" y="0" dx="2" dy="1.2" layer="1"/>
<smd name="7" x="-6.75" y="-1.8" dx="2" dy="1.2" layer="1"/>
<smd name="8" x="-6.75" y="-3.6" dx="2" dy="1.2" layer="1"/>
<smd name="9" x="-6.75" y="-5.4" dx="2" dy="1.2" layer="1"/>
<smd name="10" x="-6.75" y="-7.2" dx="2" dy="1.2" layer="1"/>
<smd name="11" x="-6.75" y="-9" dx="2" dy="1.2" layer="1"/>
<smd name="12" x="-4.5" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="13" x="-2.7" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="14" x="-0.9" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="15" x="0.9" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="16" x="2.7" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="17" x="4.5" y="-10.75" dx="2" dy="1.2" layer="1" rot="R90"/>
<smd name="18" x="6.75" y="-9" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="19" x="6.75" y="-7.2" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="20" x="6.75" y="-5.4" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="21" x="6.75" y="-3.6" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="22" x="6.75" y="-1.8" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="23" x="6.75" y="0" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="24" x="6.75" y="1.8" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="25" x="6.75" y="3.6" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="26" x="6.75" y="5.4" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="27" x="6.75" y="7.2" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="28" x="6.75" y="9" dx="2" dy="1.2" layer="1" rot="R180"/>
<smd name="29" x="4.5" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
<smd name="30" x="2.7" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
<smd name="31" x="0.9" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
<smd name="32" x="-0.9" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
<smd name="33" x="-2.7" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
<smd name="34" x="-4.5" y="10.75" dx="2" dy="1.2" layer="1" rot="R270"/>
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
<smd name="1" x="-0.762" y="0" dx="0.889" dy="0.762" layer="1" rot="R90"/>
<smd name="2" x="0.762" y="0" dx="0.889" dy="0.762" layer="1" rot="R90"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-0.8" y1="0.4" x2="0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="0.4" x2="0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="-0.4" x2="-0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-0.8" y1="-0.4" x2="-0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="-0.2" y1="0.323" x2="0.2" y2="0.323" width="0.2" layer="21"/>
<wire x1="0.2" y1="0.323" x2="0.2" y2="-0.323" width="0.2" layer="21"/>
<wire x1="0.2" y1="-0.323" x2="-0.2" y2="-0.323" width="0.2" layer="21"/>
<wire x1="-0.2" y1="-0.323" x2="-0.2" y2="0.323" width="0.2" layer="21"/>
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
<package name="ISP">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="4" x="2.54" y="-2.54" drill="1.1" diameter="1.65" rot="R270"/>
<pad name="5" x="0" y="-5.08" drill="1.1" diameter="1.65" rot="R270"/>
<pad name="6" x="2.54" y="-5.08" drill="1.1" diameter="1.65" rot="R270"/>
<text x="1.27" y="2.032" size="1.27" layer="25" align="center">&gt;NAME</text>
<pad name="1" x="0" y="0" drill="1.1" diameter="1.65" rot="R270"/>
<pad name="2" x="2.54" y="0" drill="1.1" diameter="1.65" rot="R270"/>
<pad name="3" x="0" y="-2.54" drill="1.1" diameter="1.65" rot="R270"/>
<text x="-1.778" y="2.667" size="1.27" layer="51">&gt;VALUE</text>
<wire x1="-1.25" y1="1.25" x2="3.8" y2="1.25" width="0.2" layer="21"/>
<wire x1="3.8" y1="1.25" x2="3.8" y2="-6.4" width="0.2" layer="21"/>
<wire x1="3.8" y1="-6.4" x2="-1.25" y2="-6.4" width="0.2" layer="21"/>
<wire x1="-1.25" y1="-6.4" x2="-1.25" y2="1.25" width="0.2" layer="21"/>
<wire x1="-1.25" y1="1.25" x2="3.8" y2="1.25" width="0.1" layer="51"/>
<wire x1="3.8" y1="1.25" x2="3.8" y2="-6.4" width="0.1" layer="51"/>
<wire x1="3.8" y1="-6.4" x2="-1.25" y2="-6.4" width="0.1" layer="51"/>
<wire x1="-1.25" y1="-6.4" x2="-1.25" y2="1.25" width="0.1" layer="51"/>
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
<package name="USB-MICRO-DIP-VER">
<pad name="3" x="0" y="0" drill="0.45"/>
<pad name="1" x="-1.3" y="0" drill="0.45"/>
<pad name="2" x="-0.65" y="-1" drill="0.45"/>
<pad name="4" x="0.65" y="-1" drill="0.45"/>
<pad name="5" x="1.3" y="0" drill="0.45"/>
<pad name="6" x="-3.575" y="-0.22" drill="1.5"/>
<pad name="7" x="3.575" y="-0.22" drill="1.5"/>
<wire x1="-4" y1="-0.2" x2="-4" y2="0.7" width="0.1" layer="51"/>
<wire x1="-4" y1="0.7" x2="-3.5" y2="1.2" width="0.1" layer="51"/>
<wire x1="-3.5" y1="1.2" x2="3.5" y2="1.2" width="0.1" layer="51"/>
<wire x1="3.5" y1="1.2" x2="4" y2="0.7" width="0.1" layer="51"/>
<wire x1="4" y1="0.7" x2="4" y2="-0.2" width="0.1" layer="51"/>
<wire x1="4" y1="-0.2" x2="2.5" y2="-1.7" width="0.1" layer="51"/>
<wire x1="2.5" y1="-1.7" x2="-2.5" y2="-1.7" width="0.1" layer="51"/>
<wire x1="-2.5" y1="-1.7" x2="-4" y2="-0.2" width="0.1" layer="51"/>
<wire x1="-3.4" y1="1.2" x2="3.4" y2="1.2" width="0.2" layer="21"/>
<wire x1="-2.85" y1="-1.35" x2="-2.5" y2="-1.7" width="0.2" layer="21"/>
<wire x1="-2.5" y1="-1.7" x2="2.5" y2="-1.7" width="0.2" layer="21"/>
<wire x1="2.5" y1="-1.7" x2="2.85" y2="-1.35" width="0.2" layer="21"/>
<text x="-3.5" y="1.45" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.5" y="2.75" size="1.27" layer="27">&gt;VALUE</text>
<wire x1="-6" y1="2.8" x2="6" y2="2.8" width="0.1" layer="51"/>
<wire x1="6" y1="2.8" x2="6" y2="-3.2" width="0.1" layer="51"/>
<wire x1="6" y1="-3.2" x2="-6" y2="-3.2" width="0.1" layer="51"/>
<wire x1="-6" y1="-3.2" x2="-6" y2="2.8" width="0.1" layer="51"/>
</package>
<package name="CRYSTAL-5032-4P">
<smd name="1" x="-1.9" y="-1.25" dx="2" dy="1.5" layer="1"/>
<smd name="2" x="1.9" y="-1.25" dx="2" dy="1.5" layer="1"/>
<smd name="3" x="1.9" y="1.25" dx="2" dy="1.5" layer="1"/>
<smd name="4" x="-1.9" y="1.25" dx="2" dy="1.5" layer="1"/>
<wire x1="-0.55" y1="1.6" x2="0.55" y2="1.6" width="0.2" layer="21"/>
<wire x1="0.55" y1="1.6" x2="0.55" y2="-1.6" width="0.2" layer="21"/>
<wire x1="0.55" y1="-1.6" x2="-0.55" y2="-1.6" width="0.2" layer="21"/>
<wire x1="-0.55" y1="-1.6" x2="-0.55" y2="1.6" width="0.2" layer="21"/>
<text x="-3.15" y="0.4" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.15" y="0.2" size="1.27" layer="27">&gt;VALUE</text>
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
<package name="VDFN-6">
<smd name="7" x="0" y="0" dx="1.4" dy="0.8" layer="1" rot="R90"/>
<smd name="1" x="-1.025" y="0.65" dx="0.75" dy="0.3" layer="1"/>
<smd name="2" x="-1.025" y="0" dx="0.75" dy="0.3" layer="1"/>
<smd name="3" x="-1.025" y="-0.65" dx="0.75" dy="0.3" layer="1"/>
<smd name="4" x="1.025" y="-0.65" dx="0.75" dy="0.3" layer="1"/>
<smd name="5" x="1.025" y="0" dx="0.75" dy="0.3" layer="1"/>
<smd name="6" x="1.025" y="0.65" dx="0.75" dy="0.3" layer="1"/>
<wire x1="-1" y1="1" x2="1" y2="1" width="0.2" layer="21"/>
<wire x1="-1" y1="1" x2="1" y2="1" width="0.1" layer="51"/>
<wire x1="-1" y1="-1" x2="1" y2="-1" width="0.1" layer="51"/>
<text x="-2.54" y="0" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="1.27" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="LED-0603">
<description>&lt;b&gt;ERA3A&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-0.762" y="0" dx="0.889" dy="0.762" layer="1" rot="R90"/>
<smd name="2" x="0.762" y="0" dx="0.889" dy="0.762" layer="1" rot="R90"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-0.8" y1="0.4" x2="0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="0.4" x2="0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="-0.4" x2="-0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-0.8" y1="-0.4" x2="-0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.25" y1="0.35" x2="0.25" y2="-0.35" width="0.2" layer="21"/>
<wire x1="0.25" y1="-0.35" x2="-0.25" y2="0" width="0.2" layer="21"/>
<wire x1="-0.25" y1="0" x2="0.25" y2="0.35" width="0.2" layer="21"/>
<wire x1="0.25" y1="0.35" x2="0" y2="0" width="0.2" layer="21"/>
<wire x1="0" y1="0" x2="0.2" y2="-0.05" width="0.2" layer="21"/>
<wire x1="0.2" y1="-0.05" x2="0.15" y2="-0.15" width="0.2" layer="21"/>
<wire x1="-0.25" y1="0.35" x2="-0.25" y2="-0.35" width="0.2" layer="21"/>
</package>
<package name="HEADER-10P-2R-DIP-2_54MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="4" x="7.62" y="0" drill="1.1" diameter="1.65"/>
<pad name="5" x="10.16" y="0" drill="1.1" diameter="1.65"/>
<pad name="6" x="0" y="2.54" drill="1.1" diameter="1.65"/>
<pad name="1" x="0" y="0" drill="1.1" diameter="1.65"/>
<pad name="2" x="2.54" y="0" drill="1.1" diameter="1.65"/>
<pad name="3" x="5.08" y="0" drill="1.1" diameter="1.65"/>
<text x="-1.278" y="5.217" size="1.27" layer="51">&gt;VALUE</text>
<pad name="7" x="2.54" y="2.54" drill="1.1" diameter="1.65"/>
<pad name="8" x="5.08" y="2.54" drill="1.1" diameter="1.65"/>
<wire x1="-1.25" y1="-1.25" x2="11.39" y2="-1.25" width="0.2" layer="21"/>
<wire x1="11.39" y1="-1.25" x2="11.39" y2="3.8" width="0.2" layer="21"/>
<wire x1="11.39" y1="3.8" x2="-1.25" y2="3.8" width="0.2" layer="21"/>
<wire x1="-1.25" y1="3.8" x2="-1.25" y2="-1.25" width="0.2" layer="21"/>
<text x="-1.3" y="3.95" size="1.27" layer="21">&gt;NAME</text>
<wire x1="-1.25" y1="3.8" x2="-1.25" y2="-1.25" width="0.1" layer="255"/>
<wire x1="-1.25" y1="-1.25" x2="11.39" y2="-1.25" width="0.1" layer="255"/>
<wire x1="11.39" y1="-1.25" x2="11.39" y2="3.8" width="0.1" layer="255"/>
<wire x1="11.39" y1="3.8" x2="-1.25" y2="3.8" width="0.1" layer="255"/>
<pad name="9" x="7.62" y="2.54" drill="1.1" diameter="1.65"/>
<pad name="10" x="10.16" y="2.54" drill="1.1" diameter="1.65"/>
</package>
<package name="BUTTON-4P-SMD-5X5">
<description>&lt;b&gt;COM-08720-1&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-3.1" y="1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="4" x="3.1" y="1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="2" x="-3.1" y="-1.85" dx="1.8" dy="1.1" layer="1"/>
<smd name="3" x="3.1" y="-1.85" dx="1.8" dy="1.1" layer="1"/>
<text x="-0.5" y="3.8" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-2.6" y1="1" x2="-2.6" y2="-1" width="0.1" layer="51"/>
<wire x1="-2.6" y1="-1" x2="-1" y2="-2.6" width="0.1" layer="51"/>
<wire x1="-1" y1="-2.6" x2="1" y2="-2.6" width="0.1" layer="51"/>
<wire x1="1" y1="-2.6" x2="2.6" y2="-1" width="0.1" layer="51"/>
<wire x1="2.6" y1="-1" x2="2.6" y2="1" width="0.2" layer="51"/>
<wire x1="2.6" y1="1" x2="1" y2="2.6" width="0.1" layer="51"/>
<wire x1="1" y1="2.6" x2="-1" y2="2.6" width="0.2" layer="51"/>
<wire x1="-1" y1="2.6" x2="-2.6" y2="1" width="0.1" layer="51"/>
<wire x1="-2.6" y1="1" x2="-2.6" y2="-1" width="0.2" layer="21"/>
<wire x1="-1" y1="-2.6" x2="1" y2="-2.6" width="0.2" layer="21"/>
<wire x1="2.6" y1="-1" x2="2.6" y2="1" width="0.1" layer="21"/>
<wire x1="-1" y1="2.6" x2="1" y2="2.6" width="0.1" layer="21"/>
<wire x1="1" y1="-2.6" x2="1.8" y2="-1.8" width="0.2" layer="21"/>
<wire x1="1" y1="2.6" x2="1.8" y2="1.8" width="0.2" layer="21"/>
<wire x1="-1" y1="2.6" x2="-1.8" y2="1.8" width="0.2" layer="21"/>
<wire x1="-1" y1="-2.6" x2="-1.85" y2="-1.75" width="0.2" layer="21"/>
</package>
<package name="HEADER-6P-2R-DIP-2_54MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="4" x="-2.54" y="1.27" drill="1.1" diameter="1.65"/>
<pad name="5" x="0" y="1.27" drill="1.1" diameter="1.65"/>
<pad name="6" x="2.54" y="1.27" drill="1.1" diameter="1.65"/>
<text x="1.27" y="2.032" size="1.27" layer="25" align="center">&gt;NAME</text>
<pad name="1" x="-2.54" y="-1.27" drill="1.1" diameter="1.65"/>
<pad name="2" x="0" y="-1.27" drill="1.1" diameter="1.65"/>
<pad name="3" x="2.54" y="-1.27" drill="1.1" diameter="1.65"/>
<text x="-1.778" y="2.667" size="1.27" layer="51">&gt;VALUE</text>
<wire x1="-3.8" y1="2.55" x2="3.8" y2="2.55" width="0.1" layer="51"/>
<wire x1="3.8" y1="2.55" x2="3.8" y2="-2.55" width="0.1" layer="51"/>
<wire x1="3.8" y1="-2.55" x2="-3.8" y2="-2.55" width="0.1" layer="51"/>
<wire x1="-3.8" y1="-2.55" x2="-3.8" y2="2.55" width="0.1" layer="51"/>
<wire x1="-3.8" y1="2.55" x2="3.8" y2="2.55" width="0.2" layer="21"/>
<wire x1="3.8" y1="2.55" x2="3.8" y2="-2.55" width="0.2" layer="21"/>
<wire x1="3.8" y1="-2.55" x2="-3.8" y2="-2.55" width="0.2" layer="21"/>
<wire x1="-3.8" y1="-2.55" x2="-3.8" y2="2.55" width="0.2" layer="21"/>
</package>
</packages>
<symbols>
<symbol name="ATMEGA32U4">
<wire x1="10.16" y1="17.78" x2="71.12" y2="17.78" width="0.254" layer="94"/>
<wire x1="71.12" y1="-43.18" x2="71.12" y2="17.78" width="0.254" layer="94"/>
<wire x1="71.12" y1="-43.18" x2="10.16" y2="-43.18" width="0.254" layer="94"/>
<wire x1="10.16" y1="17.78" x2="10.16" y2="-43.18" width="0.254" layer="94"/>
<text x="11.43" y="20.32" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="11.43" y="22.86" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="PE6/AIN0/INT6" x="5.08" y="0" length="middle"/>
<pin name="UVCC" x="5.08" y="-2.54" length="middle" direction="pwr"/>
<pin name="D-" x="5.08" y="-5.08" length="middle"/>
<pin name="D+" x="5.08" y="-7.62" length="middle"/>
<pin name="UGND" x="5.08" y="-10.16" length="middle" direction="pwr"/>
<pin name="UCAP" x="5.08" y="-12.7" length="middle"/>
<pin name="VBUS" x="5.08" y="-15.24" length="middle" direction="pwr"/>
<pin name="PB0/PCINT0" x="5.08" y="-17.78" length="middle"/>
<pin name="PB1/SCLK/PCINT1" x="5.08" y="-20.32" length="middle"/>
<pin name="PB2/MOSI/PCINT2" x="5.08" y="-22.86" length="middle"/>
<pin name="PB3/MISO/PCINT3" x="5.08" y="-25.4" length="middle"/>
<pin name="PB7/PCINT7" x="27.94" y="-48.26" length="middle" rot="R90"/>
<pin name="!RESET" x="30.48" y="-48.26" length="middle" rot="R90"/>
<pin name="VCC" x="33.02" y="-48.26" length="middle" direction="pwr" rot="R90"/>
<pin name="GND" x="35.56" y="-48.26" length="middle" direction="pwr" rot="R90"/>
<pin name="XTAL2" x="38.1" y="-48.26" length="middle" rot="R90"/>
<pin name="XTAL1" x="40.64" y="-48.26" length="middle" rot="R90"/>
<pin name="PD0/SCL/INT0" x="43.18" y="-48.26" length="middle" rot="R90"/>
<pin name="PD1/SDA/INT1" x="45.72" y="-48.26" length="middle" rot="R90"/>
<pin name="PD2/RXD1/INT2" x="48.26" y="-48.26" length="middle" rot="R90"/>
<pin name="PD3/TXD1/INT3" x="50.8" y="-48.26" length="middle" rot="R90"/>
<pin name="PD5" x="53.34" y="-48.26" length="middle" rot="R90"/>
<pin name="HWB/PE2" x="76.2" y="0" length="middle" rot="R180"/>
<pin name="PC7" x="76.2" y="-2.54" length="middle" direction="in" rot="R180"/>
<pin name="PC6" x="76.2" y="-5.08" length="middle" rot="R180"/>
<pin name="PCINT6/ADC13/PB6" x="76.2" y="-7.62" length="middle" rot="R180"/>
<pin name="PCINT5/ADC12/PB5" x="76.2" y="-10.16" length="middle" rot="R180"/>
<pin name="PCINT4/ADC11/PB4" x="76.2" y="-12.7" length="middle" rot="R180"/>
<pin name="ADC10/PD7" x="76.2" y="-15.24" length="middle" rot="R180"/>
<pin name="ADC9/PD6" x="76.2" y="-17.78" length="middle" rot="R180"/>
<pin name="ADC8/PD4" x="76.2" y="-20.32" length="middle" rot="R180"/>
<pin name="AVCC" x="76.2" y="-22.86" length="middle" direction="pwr" rot="R180"/>
<pin name="GND_1" x="76.2" y="-25.4" length="middle" direction="pwr" rot="R180"/>
<pin name="AVCC_1" x="27.94" y="22.86" length="middle" direction="pwr" rot="R270"/>
<pin name="GND_3" x="30.48" y="22.86" length="middle" direction="pwr" rot="R270"/>
<pin name="AREF" x="33.02" y="22.86" length="middle" direction="in" rot="R270"/>
<pin name="ADC0/PF0" x="35.56" y="22.86" length="middle" rot="R270"/>
<pin name="ADC1/PF1" x="38.1" y="22.86" length="middle" rot="R270"/>
<pin name="ADC4/PF4" x="40.64" y="22.86" length="middle" rot="R270"/>
<pin name="ADC5/PF5" x="43.18" y="22.86" length="middle" rot="R270"/>
<pin name="ADC6/PF6" x="45.72" y="22.86" length="middle" rot="R270"/>
<pin name="ADC7/PF7" x="48.26" y="22.86" length="middle" rot="R270"/>
<pin name="GND_2" x="50.8" y="22.86" length="middle" direction="pwr" rot="R270"/>
<pin name="VCC_1" x="53.34" y="22.86" length="middle" direction="pwr" rot="R270"/>
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
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-3.048" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
<wire x1="-1.27" y1="-0.508" x2="1.27" y2="-0.508" width="0.254" layer="94"/>
<wire x1="-0.508" y1="-1.016" x2="0.508" y2="-1.016" width="0.254" layer="94"/>
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
<symbol name="VCC">
<circle x="0" y="0.762" radius="0.762" width="0.254" layer="94"/>
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="ANT">
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="ANT" x="0" y="-2.54" visible="off" length="short" rot="R90"/>
<wire x1="0" y1="0" x2="-1.016" y2="1.016" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="0" y2="1.016" width="0.254" layer="94"/>
<wire x1="0" y1="0" x2="1.016" y2="1.016" width="0.254" layer="94"/>
<pin name="GND" x="2.54" y="-2.54" visible="off" length="short" direction="pwr" rot="R90"/>
<text x="-2.032" y="3.81" size="1.778" layer="95">&gt;NAME</text>
</symbol>
<symbol name="ISP">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-7.62" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="17.78" y="0" length="middle" direction="pwr" rot="R180"/>
<pin name="3" x="0" y="-2.54" length="middle"/>
<pin name="4" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="5" x="0" y="-5.08" length="middle"/>
<pin name="6" x="17.78" y="-5.08" length="middle" direction="pwr" rot="R180"/>
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
<symbol name="CONNECTOR-5P">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="0" y="-10.16" length="middle"/>
</symbol>
<symbol name="CRYSTAL">
<wire x1="5.08" y1="2.032" x2="5.08" y2="-2.032" width="0.254" layer="94"/>
<wire x1="7.62" y1="2.032" x2="7.62" y2="-2.032" width="0.254" layer="94"/>
<text x="3.81" y="3.048" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="4.826" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="middle" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="middle" direction="pas" rot="R180"/>
<wire x1="5.588" y1="1.016" x2="7.112" y2="1.016" width="0.254" layer="94"/>
<wire x1="7.112" y1="1.016" x2="7.112" y2="-1.016" width="0.254" layer="94"/>
<wire x1="7.112" y1="-1.016" x2="5.588" y2="-1.016" width="0.254" layer="94"/>
<wire x1="5.588" y1="-1.016" x2="5.588" y2="1.016" width="0.254" layer="94"/>
<pin name="GND" x="6.35" y="-3.556" visible="off" length="short" direction="pwr" rot="R90"/>
</symbol>
<symbol name="DIODE-SCHOTTKY">
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
<wire x1="4.572" y1="0" x2="4.572" y2="0.762" width="0.254" layer="94"/>
<wire x1="4.572" y1="0.762" x2="4.064" y2="1.016" width="0.254" layer="94"/>
<wire x1="4.572" y1="0" x2="4.572" y2="-0.762" width="0.254" layer="94"/>
<wire x1="4.572" y1="-0.762" x2="5.08" y2="-1.016" width="0.254" layer="94"/>
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
<symbol name="UVCC">
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="UVCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
<rectangle x1="-1.016" y1="0" x2="1.016" y2="1.524" layer="94"/>
</symbol>
<symbol name="MIC5219-VDFN-6">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="EN" x="0" y="0" length="middle"/>
<pin name="GND" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="IN" x="0" y="-5.08" length="middle" direction="pwr"/>
<pin name="OUT" x="27.94" y="-5.08" length="middle" direction="pwr" rot="R180"/>
<pin name="NC" x="27.94" y="-2.54" length="middle" rot="R180"/>
<pin name="BYP" x="27.94" y="0" length="middle" rot="R180"/>
</symbol>
<symbol name="LED">
<text x="1.524" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="1.524" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="K" x="0" y="0" visible="off" length="short"/>
<pin name="A" x="12.7" y="0" visible="off" length="short" rot="R180"/>
<wire x1="4.572" y1="0" x2="8.128" y2="1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="1.016" x2="8.128" y2="0.762" width="0.254" layer="94"/>
<wire x1="8.128" y1="0.762" x2="8.128" y2="0" width="0.254" layer="94"/>
<wire x1="8.128" y1="0" x2="8.128" y2="-0.762" width="0.254" layer="94"/>
<wire x1="8.128" y1="-0.762" x2="8.128" y2="-1.016" width="0.254" layer="94"/>
<wire x1="8.128" y1="-1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="4.572" y1="1.016" x2="4.572" y2="0" width="0.254" layer="94"/>
<wire x1="4.572" y1="0" x2="4.572" y2="-1.016" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="4.572" y2="0" width="0.1524" layer="94"/>
<wire x1="8.128" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
<wire x1="5.08" y1="0" x2="8.128" y2="0.762" width="0.254" layer="94"/>
<wire x1="8.128" y1="0.762" x2="5.334" y2="0" width="0.254" layer="94"/>
<wire x1="5.334" y1="0" x2="8.128" y2="-0.762" width="0.254" layer="94"/>
<wire x1="8.128" y1="-0.762" x2="7.874" y2="0.762" width="0.254" layer="94"/>
<wire x1="7.874" y1="0.762" x2="5.842" y2="0" width="0.254" layer="94"/>
<wire x1="5.842" y1="0" x2="7.874" y2="-0.508" width="0.254" layer="94"/>
<wire x1="7.874" y1="-0.508" x2="7.874" y2="0.508" width="0.254" layer="94"/>
<wire x1="7.874" y1="0.508" x2="6.35" y2="0" width="0.254" layer="94"/>
<wire x1="6.35" y1="0" x2="7.62" y2="-0.254" width="0.254" layer="94"/>
<wire x1="7.62" y1="-0.254" x2="7.62" y2="0" width="0.254" layer="94"/>
<wire x1="7.62" y1="0" x2="7.62" y2="0.254" width="0.254" layer="94"/>
<wire x1="7.62" y1="0.254" x2="6.858" y2="0" width="0.254" layer="94"/>
<wire x1="6.858" y1="0" x2="7.62" y2="0" width="0.254" layer="94"/>
</symbol>
<symbol name="CONNECTOR-10P-2R">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-12.7" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-12.7" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="0" y="-10.16" length="middle"/>
<pin name="6" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="7" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="8" x="17.78" y="-5.08" length="middle" rot="R180"/>
<pin name="9" x="17.78" y="-7.62" length="middle" rot="R180"/>
<pin name="10" x="17.78" y="-10.16" length="middle" rot="R180"/>
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
<symbol name="BUTTON-4P">
<wire x1="2.54" y1="2.54" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="10.16" y2="-2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-2.54" x2="10.16" y2="0" width="0.254" layer="94"/>
<wire x1="10.16" y1="0" x2="10.16" y2="2.54" width="0.254" layer="94"/>
<wire x1="10.16" y1="-5.08" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="2.54" x2="2.54" y2="0" width="0.254" layer="94"/>
<text x="2.794" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="2.794" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="pad" length="short" direction="pas"/>
<pin name="2" x="0" y="-2.54" visible="pad" length="short" direction="pas"/>
<pin name="4" x="12.7" y="0" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="3" x="12.7" y="-2.54" visible="pad" length="short" direction="pas" rot="R180"/>
<wire x1="2.54" y1="0" x2="2.54" y2="-2.54" width="0.254" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="2.54" y2="-5.08" width="0.254" layer="94"/>
<wire x1="2.54" y1="0" x2="6.35" y2="0" width="0.1524" layer="94"/>
<wire x1="6.35" y1="0" x2="10.16" y2="0" width="0.1524" layer="94"/>
<wire x1="2.54" y1="-2.54" x2="6.35" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="10.16" y2="-2.54" width="0.1524" layer="94"/>
<wire x1="6.35" y1="0" x2="6.35" y2="-1.016" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-1.016" x2="5.588" y2="-1.778" width="0.1524" layer="94"/>
<wire x1="6.35" y1="-2.54" x2="6.35" y2="-1.778" width="0.1524" layer="94"/>
</symbol>
<symbol name="CONNECTOR-6P-2R">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-7.62" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="17.78" y="0" length="middle" rot="R180"/>
<pin name="5" x="17.78" y="-2.54" length="middle" rot="R180"/>
<pin name="6" x="17.78" y="-5.08" length="middle" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ATMEGA32U4-MU" prefix="ıC" uservalue="yes">
<gates>
<gate name="G$1" symbol="ATMEGA32U4" x="-5.08" y="0"/>
</gates>
<devices>
<device name="" package="QFN-44">
<connects>
<connect gate="G$1" pin="!RESET" pad="13"/>
<connect gate="G$1" pin="ADC0/PF0" pad="41"/>
<connect gate="G$1" pin="ADC1/PF1" pad="40"/>
<connect gate="G$1" pin="ADC10/PD7" pad="27"/>
<connect gate="G$1" pin="ADC4/PF4" pad="39"/>
<connect gate="G$1" pin="ADC5/PF5" pad="38"/>
<connect gate="G$1" pin="ADC6/PF6" pad="37"/>
<connect gate="G$1" pin="ADC7/PF7" pad="36"/>
<connect gate="G$1" pin="ADC8/PD4" pad="25"/>
<connect gate="G$1" pin="ADC9/PD6" pad="26"/>
<connect gate="G$1" pin="AREF" pad="42"/>
<connect gate="G$1" pin="AVCC" pad="24"/>
<connect gate="G$1" pin="AVCC_1" pad="44"/>
<connect gate="G$1" pin="D+" pad="4"/>
<connect gate="G$1" pin="D-" pad="3"/>
<connect gate="G$1" pin="GND" pad="15"/>
<connect gate="G$1" pin="GND_1" pad="23"/>
<connect gate="G$1" pin="GND_2" pad="35"/>
<connect gate="G$1" pin="GND_3" pad="43"/>
<connect gate="G$1" pin="HWB/PE2" pad="33"/>
<connect gate="G$1" pin="PB0/PCINT0" pad="8"/>
<connect gate="G$1" pin="PB1/SCLK/PCINT1" pad="9"/>
<connect gate="G$1" pin="PB2/MOSI/PCINT2" pad="10"/>
<connect gate="G$1" pin="PB3/MISO/PCINT3" pad="11"/>
<connect gate="G$1" pin="PB7/PCINT7" pad="12"/>
<connect gate="G$1" pin="PC6" pad="31"/>
<connect gate="G$1" pin="PC7" pad="32"/>
<connect gate="G$1" pin="PCINT4/ADC11/PB4" pad="28"/>
<connect gate="G$1" pin="PCINT5/ADC12/PB5" pad="29"/>
<connect gate="G$1" pin="PCINT6/ADC13/PB6" pad="30"/>
<connect gate="G$1" pin="PD0/SCL/INT0" pad="18"/>
<connect gate="G$1" pin="PD1/SDA/INT1" pad="19"/>
<connect gate="G$1" pin="PD2/RXD1/INT2" pad="20"/>
<connect gate="G$1" pin="PD3/TXD1/INT3" pad="21"/>
<connect gate="G$1" pin="PD5" pad="22"/>
<connect gate="G$1" pin="PE6/AIN0/INT6" pad="1"/>
<connect gate="G$1" pin="UCAP" pad="6"/>
<connect gate="G$1" pin="UGND" pad="5"/>
<connect gate="G$1" pin="UVCC" pad="2"/>
<connect gate="G$1" pin="VBUS" pad="7"/>
<connect gate="G$1" pin="VCC" pad="14"/>
<connect gate="G$1" pin="VCC_1" pad="34"/>
<connect gate="G$1" pin="XTAL1" pad="17"/>
<connect gate="G$1" pin="XTAL2" pad="16"/>
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
<deviceset name="ISP" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="ISP" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ISP">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
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
<deviceset name="USB-MICRO-DIP-VER" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-5P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="USB-MICRO-DIP-VER">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5 6 7"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="CRYSTAL-5032-4P" prefix="XT" uservalue="yes">
<gates>
<gate name="G$1" symbol="CRYSTAL" x="0" y="0"/>
</gates>
<devices>
<device name="" package="CRYSTAL-5032-4P">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="3"/>
<connect gate="G$1" pin="GND" pad="2 4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="DIODE-SCHOTTKY" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="DIODE-SCHOTTKY" x="0" y="0"/>
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
<deviceset name="UVCC">
<gates>
<gate name="G$1" symbol="UVCC" x="0" y="2.54"/>
</gates>
<devices>
<device name="">
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="MIC5219-VDFN-6" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="MIC5219-VDFN-6" x="0" y="0"/>
</gates>
<devices>
<device name="" package="VDFN-6">
<connects>
<connect gate="G$1" pin="BYP" pad="6"/>
<connect gate="G$1" pin="EN" pad="1"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="IN" pad="3"/>
<connect gate="G$1" pin="NC" pad="5"/>
<connect gate="G$1" pin="OUT" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LED-0603" prefix="LED" uservalue="yes">
<gates>
<gate name="G$1" symbol="LED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LED-0603">
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
<deviceset name="HEADER-10P-2R-DIP-2_54MM" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-10P-2R" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HEADER-10P-2R-DIP-2_54MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
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
<deviceset name="BUTTON-4P-SMD-5X5" prefix="SW" uservalue="yes">
<gates>
<gate name="G$1" symbol="BUTTON-4P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BUTTON-4P-SMD-5X5">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="HEADER-6P-2R-DIP-2_54MM" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-6P-2R" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HEADER-6P-2R-DIP-2_54MM">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
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
<part name="IC1" library="SamacSys_Parts" deviceset="ATMEGA32U4-MU" device=""/>
<part name="MOD1" library="SamacSys_Parts" deviceset="RAK811" device=""/>
<part name="R11" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND6" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND7" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C11" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="ANT1" library="SamacSys_Parts" deviceset="SMA-RF" device=""/>
<part name="GND8" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C10" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND9" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND10" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R8" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R6" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="CON1" library="SamacSys_Parts" deviceset="ISP" device=""/>
<part name="GND11" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP5" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R4" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C8" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND12" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP6" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="IC2" library="SamacSys_Parts" deviceset="S-5712-SOT23-3" device=""/>
<part name="C7" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND13" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP7" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND14" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="CON4" library="SamacSys_Parts" deviceset="USB-MICRO-DIP-VER" device=""/>
<part name="XT1" library="SamacSys_Parts" deviceset="CRYSTAL-5032-4P" device=""/>
<part name="GND15" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP8" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND16" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND17" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C3" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="C4" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND18" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND19" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND20" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND21" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND22" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND23" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R10" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="CON3" library="SamacSys_Parts" deviceset="JST-PH-2-2.0MM" device=""/>
<part name="R12" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND39" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="F1" library="SamacSys_Parts" deviceset="FUSE-0805" device=""/>
<part name="D1" library="SamacSys_Parts" deviceset="DIODE-SCHOTTKY" device=""/>
<part name="GND40" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C14" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="C15" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="C16" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND42" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND43" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND44" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND46" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="C9" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="D2" library="SamacSys_Parts" deviceset="DIODE-SCHOTTKY" device=""/>
<part name="C17" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND45" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R18" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R19" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C5" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND48" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP18" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP19" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="U$1" library="SamacSys_Parts" deviceset="UVCC" device=""/>
<part name="U$2" library="SamacSys_Parts" deviceset="UVCC" device=""/>
<part name="U$3" library="SamacSys_Parts" deviceset="UVCC" device=""/>
<part name="GND49" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C6" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND50" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R3" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP20" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP21" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R2" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND51" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R16" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R17" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND53" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R15" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C12" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND54" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="Q3" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="Q2" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="R14" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP15" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R13" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND33" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="Q1" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="GND52" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="IC3" library="SamacSys_Parts" deviceset="MIC5219-VDFN-6" device=""/>
<part name="C18" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="LED1" library="SamacSys_Parts" deviceset="LED-0603" device=""/>
<part name="R9" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND56" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R5" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R7" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP22" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="CON2" library="SamacSys_Parts" deviceset="HEADER-10P-2R-DIP-2_54MM" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND58" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND57" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP17" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="D3" library="SamacSys_Parts" deviceset="ZENER-SOD123" device=""/>
<part name="GND59" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP12" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND60" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SW1" library="SamacSys_Parts" deviceset="BUTTON-4P-SMD-5X5" device=""/>
<part name="GND55" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="CON5" library="SamacSys_Parts" deviceset="HEADER-6P-2R-DIP-2_54MM" device=""/>
<part name="R20" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R21" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R22" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R23" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND24" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND25" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP3" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP4" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP9" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="C13" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND26" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="IC1" gate="G$1" x="-40.64" y="12.7" smashed="yes">
<attribute name="NAME" x="-29.464" y="32.258" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-29.21" y="35.56" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="MOD1" gate="G$1" x="73.66" y="20.32" smashed="yes">
<attribute name="NAME" x="80.01" y="32.004" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="80.01" y="33.274" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R11" gate="G$1" x="127" y="10.16" smashed="yes" rot="R90">
<attribute name="NAME" x="124.968" y="14.224" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="123.19" y="13.208" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND1" gate="1" x="119.38" y="20.32" smashed="yes" rot="R90">
<attribute name="VALUE" x="122.428" y="17.78" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND2" gate="1" x="71.12" y="20.32" smashed="yes" rot="R270">
<attribute name="VALUE" x="68.072" y="22.86" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND3" gate="1" x="104.14" y="35.56" smashed="yes" rot="R90">
<attribute name="VALUE" x="107.188" y="33.02" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND4" gate="1" x="119.38" y="2.54" smashed="yes" rot="R90">
<attribute name="VALUE" x="122.428" y="0" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND5" gate="1" x="88.9" y="-22.86" smashed="yes">
<attribute name="VALUE" x="86.36" y="-25.908" size="1.778" layer="96"/>
</instance>
<instance part="GND6" gate="1" x="86.36" y="35.56" smashed="yes" rot="R270">
<attribute name="VALUE" x="83.312" y="38.1" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND7" gate="1" x="127" y="-5.08" smashed="yes">
<attribute name="VALUE" x="124.46" y="-8.128" size="1.778" layer="96"/>
</instance>
<instance part="C11" gate="G$1" x="127" y="-2.54" smashed="yes" rot="R90">
<attribute name="NAME" x="130.048" y="1.778" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="122.174" y="1.27" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="ANT1" gate="G$1" x="91.44" y="43.18" smashed="yes">
<attribute name="VALUE" x="89.535" y="45.085" size="1.778" layer="96"/>
<attribute name="NAME" x="88.9" y="44.958" size="1.778" layer="95"/>
</instance>
<instance part="GND8" gate="1" x="93.98" y="38.1" smashed="yes">
<attribute name="VALUE" x="91.44" y="35.052" size="1.778" layer="96"/>
</instance>
<instance part="C10" gate="G$1" x="68.58" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="65.532" y="-13.97" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="63.754" y="-13.97" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND9" gate="1" x="73.66" y="-20.32" smashed="yes">
<attribute name="VALUE" x="71.12" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND10" gate="1" x="68.58" y="-20.32" smashed="yes">
<attribute name="VALUE" x="66.04" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="R8" gate="G$1" x="60.96" y="7.62" smashed="yes" rot="R90">
<attribute name="NAME" x="62.992" y="12.192" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="57.15" y="10.668" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R6" gate="G$1" x="58.42" y="7.62" smashed="yes" rot="R90">
<attribute name="NAME" x="56.388" y="12.192" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="54.61" y="10.668" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="CON1" gate="G$1" x="-58.42" y="-17.78" smashed="yes">
<attribute name="NAME" x="-53.34" y="-13.716" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-53.34" y="-12.446" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND11" gate="1" x="-40.64" y="-25.4" smashed="yes">
<attribute name="VALUE" x="-43.18" y="-28.448" size="1.778" layer="96"/>
</instance>
<instance part="SUP5" gate="G$1" x="-40.64" y="-15.24" smashed="yes">
<attribute name="VALUE" x="-42.545" y="-13.335" size="1.778" layer="96"/>
</instance>
<instance part="R4" gate="G$1" x="-20.32" y="-60.96" smashed="yes" rot="R90">
<attribute name="NAME" x="-22.352" y="-56.388" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-24.13" y="-57.912" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C8" gate="G$1" x="-25.4" y="-60.96" smashed="yes" rot="R90">
<attribute name="NAME" x="-28.448" y="-56.134" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-30.226" y="-57.15" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND12" gate="1" x="-25.4" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-27.94" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="SUP6" gate="G$1" x="-20.32" y="-63.5" smashed="yes" rot="R180">
<attribute name="VALUE" x="-18.415" y="-65.405" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="IC2" gate="G$1" x="-38.1" y="-48.26" smashed="yes">
<attribute name="NAME" x="-55.88" y="-39.116" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-55.88" y="-37.846" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C7" gate="G$1" x="-60.96" y="-60.96" smashed="yes" rot="R90">
<attribute name="NAME" x="-64.008" y="-56.134" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-65.786" y="-57.15" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND13" gate="1" x="-48.26" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-50.8" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="SUP7" gate="G$1" x="-63.5" y="-35.56" smashed="yes" rot="R90">
<attribute name="VALUE" x="-65.405" y="-37.465" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND14" gate="1" x="-60.96" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-63.5" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="CON4" gate="G$1" x="-142.24" y="40.64" smashed="yes" rot="MR0">
<attribute name="NAME" x="-147.32" y="44.196" size="1.778" layer="95" rot="MR0" align="center-left"/>
<attribute name="VALUE" x="-147.32" y="45.974" size="1.778" layer="96" rot="MR0" align="center-left"/>
</instance>
<instance part="XT1" gate="G$1" x="0" y="-63.5" smashed="yes" rot="R90">
<attribute name="NAME" x="3.556" y="-59.182" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-4.826" y="-59.69" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND15" gate="1" x="-5.08" y="-38.1" smashed="yes">
<attribute name="VALUE" x="-7.62" y="-41.148" size="1.778" layer="96"/>
</instance>
<instance part="SUP8" gate="G$1" x="-15.24" y="-48.26" smashed="yes" rot="R90">
<attribute name="VALUE" x="-17.145" y="-50.165" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="C2" gate="G$1" x="-12.7" y="-60.96" smashed="yes" rot="R90">
<attribute name="NAME" x="-16.002" y="-56.134" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-17.526" y="-57.15" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND16" gate="1" x="-12.7" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-15.24" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="GND17" gate="1" x="-7.62" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-10.16" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="C3" gate="G$1" x="12.7" y="-63.5" smashed="yes" rot="R180">
<attribute name="NAME" x="7.874" y="-66.548" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="8.89" y="-68.326" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="C4" gate="G$1" x="12.7" y="-50.8" smashed="yes" rot="R180">
<attribute name="NAME" x="7.874" y="-53.848" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="8.89" y="-55.626" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="GND18" gate="1" x="15.24" y="-63.5" smashed="yes" rot="R90">
<attribute name="VALUE" x="18.288" y="-66.04" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND19" gate="1" x="15.24" y="-50.8" smashed="yes" rot="R90">
<attribute name="VALUE" x="18.288" y="-53.34" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND20" gate="1" x="35.56" y="-15.24" smashed="yes">
<attribute name="VALUE" x="33.02" y="-18.288" size="1.778" layer="96"/>
</instance>
<instance part="GND21" gate="1" x="10.16" y="38.1" smashed="yes" rot="R180">
<attribute name="VALUE" x="12.7" y="41.148" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND22" gate="1" x="-10.16" y="38.1" smashed="yes" rot="R180">
<attribute name="VALUE" x="-7.62" y="41.148" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND23" gate="1" x="-38.1" y="2.54" smashed="yes" rot="R270">
<attribute name="VALUE" x="-41.148" y="5.08" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R10" gate="G$1" x="121.92" y="10.16" smashed="yes" rot="R270">
<attribute name="NAME" x="123.952" y="6.096" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="125.73" y="7.112" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="CON3" gate="G$1" x="-203.2" y="40.64" smashed="yes" rot="MR0">
<attribute name="NAME" x="-209.296" y="44.704" size="1.778" layer="95" rot="MR0" align="center-left"/>
<attribute name="VALUE" x="-208.28" y="45.974" size="1.778" layer="96" rot="MR0" align="center-left"/>
</instance>
<instance part="R12" gate="G$1" x="-198.12" y="12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="-196.088" y="16.764" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-201.93" y="15.748" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND39" gate="1" x="-203.2" y="33.02" smashed="yes">
<attribute name="VALUE" x="-205.74" y="29.972" size="1.778" layer="96"/>
</instance>
<instance part="F1" gate="G$1" x="-142.24" y="40.64" smashed="yes">
<attribute name="NAME" x="-138.684" y="42.672" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-139.7" y="44.45" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="D1" gate="G$1" x="-116.84" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="-121.92" y="43.18" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-118.364" y="36.83" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="GND40" gate="1" x="-114.3" y="38.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="-117.348" y="40.64" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C14" gate="G$1" x="-116.84" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-119.888" y="26.67" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-121.666" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C15" gate="G$1" x="-81.28" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-84.328" y="26.67" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-86.106" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C16" gate="G$1" x="-68.58" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-70.104" y="30.734" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-73.406" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND42" gate="1" x="-116.84" y="20.32" smashed="yes">
<attribute name="VALUE" x="-119.38" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="GND43" gate="1" x="-53.34" y="20.32" smashed="yes">
<attribute name="VALUE" x="-55.88" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="GND44" gate="1" x="-81.28" y="20.32" smashed="yes">
<attribute name="VALUE" x="-83.82" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="GND46" gate="1" x="-68.58" y="20.32" smashed="yes">
<attribute name="VALUE" x="-71.12" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="C1" gate="G$1" x="-7.62" y="-60.96" smashed="yes" rot="R90">
<attribute name="NAME" x="-4.318" y="-56.134" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-12.446" y="-57.15" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C9" gate="G$1" x="73.66" y="-17.78" smashed="yes" rot="R90">
<attribute name="NAME" x="76.708" y="-12.954" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="68.834" y="-13.97" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="D2" gate="G$1" x="-68.58" y="40.64" smashed="yes" rot="R180">
<attribute name="NAME" x="-73.66" y="42.926" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="-70.104" y="36.83" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="C17" gate="G$1" x="-63.5" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-65.024" y="30.734" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-68.326" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND45" gate="1" x="-63.5" y="20.32" smashed="yes">
<attribute name="VALUE" x="-66.04" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="R18" gate="G$1" x="-142.24" y="38.1" smashed="yes">
<attribute name="NAME" x="-131.064" y="39.116" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-139.192" y="41.91" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R19" gate="G$1" x="-142.24" y="35.56" smashed="yes">
<attribute name="NAME" x="-131.318" y="36.576" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-139.192" y="39.37" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C5" gate="G$1" x="-20.32" y="45.72" smashed="yes">
<attribute name="NAME" x="-15.494" y="48.768" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-16.51" y="50.546" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND48" gate="1" x="-22.86" y="45.72" smashed="yes" rot="R270">
<attribute name="VALUE" x="-25.908" y="48.26" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP18" gate="G$1" x="-12.7" y="38.1" smashed="yes">
<attribute name="VALUE" x="-14.605" y="40.005" size="1.778" layer="96"/>
</instance>
<instance part="SUP19" gate="G$1" x="12.7" y="38.1" smashed="yes">
<attribute name="VALUE" x="10.795" y="40.005" size="1.778" layer="96"/>
</instance>
<instance part="U$1" gate="G$1" x="-38.1" y="10.16" smashed="yes" rot="R90">
<attribute name="VALUE" x="-40.005" y="8.255" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U$2" gate="G$1" x="-38.1" y="-2.54" smashed="yes" rot="R90">
<attribute name="VALUE" x="-40.005" y="-4.445" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="U$3" gate="G$1" x="-127" y="45.72" smashed="yes" rot="R270">
<attribute name="VALUE" x="-124.587" y="45.085" size="1.778" layer="96"/>
</instance>
<instance part="GND49" gate="1" x="-142.24" y="27.94" smashed="yes">
<attribute name="VALUE" x="-144.78" y="24.892" size="1.778" layer="96"/>
</instance>
<instance part="C6" gate="G$1" x="-53.34" y="-2.54" smashed="yes">
<attribute name="NAME" x="-48.514" y="1.016" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-49.53" y="2.286" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND50" gate="1" x="-53.34" y="-5.08" smashed="yes">
<attribute name="VALUE" x="-55.88" y="-8.128" size="1.778" layer="96"/>
</instance>
<instance part="R3" gate="G$1" x="45.72" y="27.94" smashed="yes" rot="R90">
<attribute name="NAME" x="43.688" y="32.004" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="41.91" y="30.988" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP20" gate="G$1" x="45.72" y="43.18" smashed="yes">
<attribute name="VALUE" x="43.815" y="45.085" size="1.778" layer="96"/>
</instance>
<instance part="SUP21" gate="G$1" x="38.1" y="-10.16" smashed="yes" rot="R270">
<attribute name="VALUE" x="40.005" y="-8.255" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R2" gate="G$1" x="40.64" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="38.608" y="19.304" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="36.83" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R1" gate="G$1" x="40.64" y="27.94" smashed="yes" rot="R90">
<attribute name="NAME" x="38.608" y="32.004" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="36.83" y="30.988" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND51" gate="1" x="40.64" y="12.7" smashed="yes">
<attribute name="VALUE" x="38.1" y="9.652" size="1.778" layer="96"/>
</instance>
<instance part="R16" gate="G$1" x="-180.34" y="40.64" smashed="yes">
<attribute name="NAME" x="-176.276" y="42.672" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-177.292" y="44.45" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R17" gate="G$1" x="-167.64" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-169.672" y="26.924" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-171.45" y="25.908" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND53" gate="1" x="-167.64" y="20.32" smashed="yes">
<attribute name="VALUE" x="-170.18" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="R15" gate="G$1" x="-187.96" y="40.64" smashed="yes" rot="R270">
<attribute name="NAME" x="-185.42" y="36.576" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-184.15" y="37.592" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="C12" gate="G$1" x="-162.56" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-165.608" y="26.67" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-167.386" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND54" gate="1" x="-162.56" y="20.32" smashed="yes">
<attribute name="VALUE" x="-165.1" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="Q3" gate="G$1" x="-187.96" y="22.86" smashed="yes">
<attribute name="NAME" x="-186.944" y="29.464" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-182.372" y="24.638" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="Q2" gate="G$1" x="-198.12" y="22.86" smashed="yes">
<attribute name="NAME" x="-192.532" y="26.416" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-192.532" y="24.638" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R14" gate="G$1" x="-187.96" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-189.992" y="19.304" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-191.77" y="18.288" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP15" gate="G$1" x="-193.04" y="17.78" smashed="yes" rot="R180">
<attribute name="VALUE" x="-191.135" y="15.875" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="R13" gate="G$1" x="-200.66" y="12.7" smashed="yes" rot="R90">
<attribute name="NAME" x="-202.692" y="16.764" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-204.47" y="15.748" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND33" gate="1" x="-200.66" y="10.16" smashed="yes">
<attribute name="VALUE" x="-203.2" y="7.112" size="1.778" layer="96"/>
</instance>
<instance part="Q1" gate="G$1" x="-200.66" y="35.56" smashed="yes" rot="MR90">
<attribute name="NAME" x="-196.596" y="42.164" size="1.778" layer="95" rot="MR0" align="center-left"/>
<attribute name="VALUE" x="-198.882" y="41.148" size="1.778" layer="96" rot="MR90" align="center-left"/>
</instance>
<instance part="GND52" gate="1" x="-195.58" y="33.02" smashed="yes">
<attribute name="VALUE" x="-198.12" y="29.972" size="1.778" layer="96"/>
</instance>
<instance part="IC3" gate="G$1" x="-111.76" y="40.64" smashed="yes">
<attribute name="NAME" x="-106.426" y="44.704" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-106.426" y="45.974" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C18" gate="G$1" x="-53.34" y="22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="-54.864" y="30.734" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-58.166" y="26.67" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="LED1" gate="G$1" x="45.72" y="15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="48.26" y="18.796" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="41.91" y="16.764" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R9" gate="G$1" x="101.6" y="-25.4" smashed="yes">
<attribute name="NAME" x="106.172" y="-23.368" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="104.648" y="-21.59" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND56" gate="1" x="116.84" y="-25.4" smashed="yes" rot="R90">
<attribute name="VALUE" x="119.888" y="-27.94" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="R5" gate="G$1" x="58.42" y="-10.16" smashed="yes" rot="R90">
<attribute name="NAME" x="56.388" y="-5.588" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="54.61" y="-7.112" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R7" gate="G$1" x="60.96" y="-10.16" smashed="yes" rot="R90">
<attribute name="NAME" x="62.992" y="-5.08" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="57.15" y="-7.112" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP22" gate="G$1" x="40.64" y="43.18" smashed="yes">
<attribute name="VALUE" x="38.735" y="45.085" size="1.778" layer="96"/>
</instance>
<instance part="CON2" gate="G$1" x="55.88" y="-40.64" smashed="yes">
<attribute name="NAME" x="60.96" y="-37.084" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="60.96" y="-35.306" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP2" gate="G$1" x="53.34" y="-45.72" smashed="yes" rot="R90">
<attribute name="VALUE" x="51.435" y="-47.625" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND58" gate="1" x="76.2" y="-48.26" smashed="yes" rot="R90">
<attribute name="VALUE" x="79.248" y="-50.8" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND57" gate="1" x="53.34" y="-48.26" smashed="yes" rot="R270">
<attribute name="VALUE" x="50.292" y="-45.72" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP17" gate="G$1" x="53.34" y="-50.8" smashed="yes" rot="R90">
<attribute name="VALUE" x="51.435" y="-52.705" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="D3" gate="G$1" x="-58.42" y="35.56" smashed="yes" rot="R270">
<attribute name="NAME" x="-59.944" y="35.052" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-54.61" y="34.036" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="GND59" gate="1" x="-58.42" y="20.32" smashed="yes">
<attribute name="VALUE" x="-60.96" y="17.272" size="1.778" layer="96"/>
</instance>
<instance part="SUP12" gate="G$1" x="-58.42" y="43.18" smashed="yes">
<attribute name="VALUE" x="-60.325" y="45.085" size="1.778" layer="96"/>
</instance>
<instance part="GND60" gate="1" x="6.35" y="-57.15" smashed="yes" rot="R90">
<attribute name="VALUE" x="9.398" y="-59.69" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SW1" gate="G$1" x="-43.18" y="-63.5" smashed="yes">
<attribute name="NAME" x="-40.386" y="-59.944" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-40.386" y="-58.166" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND55" gate="1" x="-27.94" y="-66.04" smashed="yes" rot="R90">
<attribute name="VALUE" x="-24.892" y="-68.58" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="CON5" gate="G$1" x="-91.44" y="-60.96" smashed="yes" rot="MR90">
<attribute name="NAME" x="-87.884" y="-55.88" size="1.778" layer="95" rot="MR90" align="center-left"/>
<attribute name="VALUE" x="-86.106" y="-55.88" size="1.778" layer="96" rot="MR90" align="center-left"/>
</instance>
<instance part="R20" gate="G$1" x="-86.36" y="-35.56" smashed="yes">
<attribute name="NAME" x="-75.946" y="-34.544" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-83.312" y="-31.75" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R21" gate="G$1" x="-86.36" y="-38.1" smashed="yes">
<attribute name="NAME" x="-75.946" y="-37.084" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-83.312" y="-34.29" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R22" gate="G$1" x="-86.36" y="-40.64" smashed="yes">
<attribute name="NAME" x="-75.946" y="-39.624" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-83.312" y="-36.83" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R23" gate="G$1" x="-86.36" y="-43.18" smashed="yes">
<attribute name="NAME" x="-75.946" y="-42.164" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-83.312" y="-39.37" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND24" gate="1" x="-93.98" y="-63.5" smashed="yes">
<attribute name="VALUE" x="-96.52" y="-66.548" size="1.778" layer="96"/>
</instance>
<instance part="GND25" gate="1" x="-71.12" y="-40.64" smashed="yes" rot="R90">
<attribute name="VALUE" x="-68.072" y="-43.18" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP3" gate="G$1" x="-71.12" y="-35.56" smashed="yes" rot="R270">
<attribute name="VALUE" x="-69.215" y="-33.655" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP4" gate="G$1" x="-71.12" y="-38.1" smashed="yes" rot="R270">
<attribute name="VALUE" x="-69.215" y="-36.195" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP9" gate="G$1" x="-71.12" y="-43.18" smashed="yes" rot="R270">
<attribute name="VALUE" x="-69.215" y="-41.275" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C13" gate="G$1" x="-144.78" y="45.72" smashed="yes">
<attribute name="NAME" x="-140.97" y="48.768" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-140.97" y="50.546" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND26" gate="1" x="-147.32" y="45.72" smashed="yes" rot="R270">
<attribute name="VALUE" x="-150.368" y="48.26" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP1" gate="G$1" x="-96.52" y="-63.5" smashed="yes" rot="R180">
<attribute name="VALUE" x="-94.615" y="-65.405" size="1.778" layer="96" rot="R180"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="GND" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_4"/>
<pinref part="GND1" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_1"/>
<pinref part="GND2" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_5"/>
<pinref part="GND3" gate="1" pin="GND"/>
<pinref part="MOD1" gate="G$1" pin="GND_8"/>
<pinref part="MOD1" gate="G$1" pin="GND_7"/>
<wire x1="93.98" y1="35.56" x2="96.52" y2="35.56" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="GND_6"/>
<wire x1="96.52" y1="35.56" x2="99.06" y2="35.56" width="0.1524" layer="91"/>
<junction x="96.52" y="35.56"/>
<wire x1="99.06" y1="35.56" x2="101.6" y2="35.56" width="0.1524" layer="91"/>
<junction x="99.06" y="35.56"/>
<junction x="101.6" y="35.56"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_3"/>
<pinref part="GND4" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_2"/>
<pinref part="GND5" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND_9"/>
<pinref part="GND6" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C11" gate="G$1" pin="1"/>
<pinref part="GND7" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="ANT1" gate="G$1" pin="GND"/>
<pinref part="GND8" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C10" gate="G$1" pin="1"/>
<pinref part="GND10" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="6"/>
<pinref part="GND11" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C8" gate="G$1" pin="1"/>
<pinref part="GND12" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC2" gate="G$1" pin="VSS"/>
<pinref part="GND13" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C7" gate="G$1" pin="1"/>
<pinref part="GND14" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="1"/>
<pinref part="GND16" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C3" gate="G$1" pin="1"/>
<pinref part="GND18" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C4" gate="G$1" pin="1"/>
<pinref part="GND19" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND_1"/>
<pinref part="GND20" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND_2"/>
<pinref part="GND21" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND_3"/>
<pinref part="GND22" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="UGND"/>
<pinref part="GND23" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<pinref part="GND15" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON3" gate="G$1" pin="2"/>
<pinref part="GND39" gate="1" pin="GND"/>
<wire x1="-203.2" y1="35.56" x2="-203.2" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C14" gate="G$1" pin="1"/>
<pinref part="GND42" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C15" gate="G$1" pin="1"/>
<pinref part="GND44" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C16" gate="G$1" pin="1"/>
<pinref part="GND46" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND17" gate="1" pin="GND"/>
<pinref part="C1" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="GND9" gate="1" pin="GND"/>
<pinref part="C9" gate="G$1" pin="1"/>
</segment>
<segment>
<pinref part="C17" gate="G$1" pin="1"/>
<pinref part="GND45" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C5" gate="G$1" pin="1"/>
<pinref part="GND48" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON4" gate="G$1" pin="5"/>
<pinref part="GND49" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C6" gate="G$1" pin="1"/>
<pinref part="GND50" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R2" gate="G$1" pin="1"/>
<pinref part="GND51" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R17" gate="G$1" pin="1"/>
<pinref part="GND53" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C12" gate="G$1" pin="1"/>
<pinref part="GND54" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R13" gate="G$1" pin="1"/>
<pinref part="GND33" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="Q1" gate="G$1" pin="G"/>
<pinref part="GND52" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="GND40" gate="1" pin="GND"/>
<pinref part="IC3" gate="G$1" pin="GND"/>
</segment>
<segment>
<pinref part="C18" gate="G$1" pin="1"/>
<pinref part="GND43" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R9" gate="G$1" pin="2"/>
<pinref part="GND56" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON2" gate="G$1" pin="4"/>
<pinref part="GND57" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON2" gate="G$1" pin="9"/>
<pinref part="GND58" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="D3" gate="G$1" pin="A"/>
<pinref part="GND59" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="SW1" gate="G$1" pin="2"/>
<wire x1="-43.18" y1="-66.04" x2="-43.18" y2="-71.12" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="-71.12" x2="-30.48" y2="-71.12" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="3"/>
<wire x1="-30.48" y1="-71.12" x2="-30.48" y2="-66.04" width="0.1524" layer="91"/>
<pinref part="GND55" gate="1" pin="GND"/>
<junction x="-30.48" y="-66.04"/>
</segment>
<segment>
<pinref part="CON5" gate="G$1" pin="2"/>
<pinref part="GND24" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R22" gate="G$1" pin="2"/>
<pinref part="GND25" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C13" gate="G$1" pin="1"/>
<pinref part="GND26" gate="1" pin="GND"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RST"/>
<wire x1="116.84" y1="10.16" x2="121.92" y2="10.16" width="0.1524" layer="91"/>
<pinref part="R11" gate="G$1" pin="1"/>
<pinref part="C11" gate="G$1" pin="2"/>
<junction x="127" y="10.16"/>
<pinref part="R10" gate="G$1" pin="1"/>
<wire x1="121.92" y1="10.16" x2="127" y2="10.16" width="0.1524" layer="91"/>
<junction x="121.92" y="10.16"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="CON1" gate="G$1" pin="2"/>
<pinref part="SUP5" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R4" gate="G$1" pin="1"/>
<pinref part="SUP6" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC2" gate="G$1" pin="VDD"/>
<pinref part="C7" gate="G$1" pin="2"/>
<wire x1="-48.26" y1="-35.56" x2="-60.96" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="-35.56" x2="-60.96" y2="-48.26" width="0.1524" layer="91"/>
<pinref part="SUP7" gate="G$1" pin="VCC"/>
<junction x="-60.96" y="-35.56"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VCC"/>
<wire x1="-7.62" y1="-35.56" x2="-7.62" y2="-48.26" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="-7.62" y1="-48.26" x2="-12.7" y2="-48.26" width="0.1524" layer="91"/>
<pinref part="SUP8" gate="G$1" pin="VCC"/>
<junction x="-12.7" y="-48.26"/>
<pinref part="C1" gate="G$1" pin="2"/>
<junction x="-7.62" y="-48.26"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="AVCC_1"/>
<pinref part="SUP18" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VCC_1"/>
<pinref part="SUP19" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="2"/>
<pinref part="SUP20" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="AVCC"/>
<pinref part="SUP21" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="Q2" gate="G$1" pin="D"/>
<pinref part="SUP15" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="SUP22" gate="G$1" pin="VCC"/>
<pinref part="R1" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="CON2" gate="G$1" pin="3"/>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="SUP17" gate="G$1" pin="VCC"/>
<pinref part="CON2" gate="G$1" pin="5"/>
</segment>
<segment>
<pinref part="C16" gate="G$1" pin="2"/>
<wire x1="-68.58" y1="40.64" x2="-68.58" y2="35.56" width="0.1524" layer="91"/>
<pinref part="D2" gate="G$1" pin="K"/>
<pinref part="C17" gate="G$1" pin="2"/>
<wire x1="-68.58" y1="40.64" x2="-63.5" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-63.5" y1="40.64" x2="-63.5" y2="35.56" width="0.1524" layer="91"/>
<junction x="-68.58" y="40.64"/>
<junction x="-63.5" y="40.64"/>
<pinref part="D3" gate="G$1" pin="K"/>
<wire x1="-63.5" y1="40.64" x2="-58.42" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-58.42" y1="40.64" x2="-58.42" y2="35.56" width="0.1524" layer="91"/>
<pinref part="SUP12" gate="G$1" pin="VCC"/>
<junction x="-58.42" y="40.64"/>
</segment>
<segment>
<pinref part="R20" gate="G$1" pin="2"/>
<pinref part="SUP3" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R21" gate="G$1" pin="2"/>
<pinref part="SUP4" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R23" gate="G$1" pin="2"/>
<pinref part="SUP9" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="CON5" gate="G$1" pin="3"/>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="ANT1" gate="G$1" pin="ANT"/>
<pinref part="MOD1" gate="G$1" pin="RF_OUT"/>
<wire x1="91.44" y1="40.64" x2="91.44" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB3/MISO/PCINT3"/>
<pinref part="CON1" gate="G$1" pin="1"/>
<wire x1="-35.56" y1="-12.7" x2="-58.42" y2="-12.7" width="0.1524" layer="91"/>
<wire x1="-58.42" y1="-12.7" x2="-58.42" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PB2/MOSI/PCINT2"/>
<wire x1="-35.56" y1="-10.16" x2="-38.1" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-38.1" y1="-10.16" x2="-38.1" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="4"/>
<wire x1="-38.1" y1="-20.32" x2="-40.64" y2="-20.32" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="XTAL2"/>
<pinref part="XT1" gate="G$1" pin="1"/>
<wire x1="-2.54" y1="-35.56" x2="-2.54" y2="-63.5" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-63.5" x2="0" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="C3" gate="G$1" pin="2"/>
<junction x="0" y="-63.5"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="XTAL1"/>
<pinref part="XT1" gate="G$1" pin="2"/>
<wire x1="0" y1="-35.56" x2="0" y2="-50.8" width="0.1524" layer="91"/>
<pinref part="C4" gate="G$1" pin="2"/>
<junction x="0" y="-50.8"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<wire x1="48.26" y1="-27.94" x2="121.92" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="121.92" y1="-27.94" x2="121.92" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="IC1" gate="G$1" pin="ADC8/PD4"/>
<wire x1="48.26" y1="-27.94" x2="48.26" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="48.26" y1="-7.62" x2="35.56" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD0/SCL/INT0"/>
<wire x1="2.54" y1="-35.56" x2="2.54" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="2.54" y1="-40.64" x2="-30.48" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-30.48" y1="-40.64" x2="-30.48" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="CON5" gate="G$1" pin="5"/>
<wire x1="-30.48" y1="-30.48" x2="-93.98" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="-93.98" y1="-30.48" x2="-93.98" y2="-38.1" width="0.1524" layer="91"/>
<pinref part="R21" gate="G$1" pin="1"/>
<wire x1="-93.98" y1="-38.1" x2="-93.98" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-86.36" y1="-38.1" x2="-93.98" y2="-38.1" width="0.1524" layer="91"/>
<junction x="-93.98" y="-38.1"/>
</segment>
</net>
<net name="N$33" class="0">
<segment>
<pinref part="CON4" gate="G$1" pin="1"/>
<pinref part="F1" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$38" class="0">
<segment>
<pinref part="C14" gate="G$1" pin="2"/>
<wire x1="-116.84" y1="35.56" x2="-121.92" y2="35.56" width="0.1524" layer="91"/>
<pinref part="D1" gate="G$1" pin="K"/>
<wire x1="-116.84" y1="35.56" x2="-116.84" y2="40.64" width="0.1524" layer="91"/>
<junction x="-116.84" y="35.56"/>
<junction x="-116.84" y="40.64"/>
<wire x1="-198.12" y1="12.7" x2="-121.92" y2="12.7" width="0.1524" layer="91"/>
<wire x1="-121.92" y1="12.7" x2="-121.92" y2="35.56" width="0.1524" layer="91"/>
<pinref part="R12" gate="G$1" pin="1"/>
<pinref part="IC3" gate="G$1" pin="EN"/>
<wire x1="-111.76" y1="40.64" x2="-116.84" y2="40.64" width="0.1524" layer="91"/>
<pinref part="IC3" gate="G$1" pin="IN"/>
<wire x1="-111.76" y1="35.56" x2="-116.84" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$40" class="0">
<segment>
<pinref part="C15" gate="G$1" pin="2"/>
<wire x1="-81.28" y1="35.56" x2="-81.28" y2="40.64" width="0.1524" layer="91"/>
<pinref part="D2" gate="G$1" pin="A"/>
<pinref part="IC3" gate="G$1" pin="OUT"/>
<wire x1="-83.82" y1="35.56" x2="-81.28" y2="35.56" width="0.1524" layer="91"/>
<junction x="-81.28" y="35.56"/>
</segment>
</net>
<net name="N$41" class="0">
<segment>
<pinref part="CON4" gate="G$1" pin="2"/>
<pinref part="R18" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$42" class="0">
<segment>
<pinref part="CON4" gate="G$1" pin="3"/>
<pinref part="R19" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$43" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="D-"/>
<wire x1="-35.56" y1="7.62" x2="-124.46" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-124.46" y1="7.62" x2="-124.46" y2="38.1" width="0.1524" layer="91"/>
<pinref part="R18" gate="G$1" pin="2"/>
<wire x1="-124.46" y1="38.1" x2="-129.54" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$44" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="D+"/>
<wire x1="-35.56" y1="5.08" x2="-127" y2="5.08" width="0.1524" layer="91"/>
<pinref part="R19" gate="G$1" pin="2"/>
<wire x1="-127" y1="5.08" x2="-127" y2="35.56" width="0.1524" layer="91"/>
<wire x1="-127" y1="35.56" x2="-129.54" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$45" class="0">
<segment>
<pinref part="CON1" gate="G$1" pin="3"/>
<wire x1="-60.96" y1="-20.32" x2="-58.42" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="-7.62" x2="-60.96" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="IC1" gate="G$1" pin="PB1/SCLK/PCINT1"/>
<wire x1="-35.56" y1="-7.62" x2="-60.96" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$31" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="AREF"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="-7.62" y1="35.56" x2="-7.62" y2="45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="UVCC" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="UVCC"/>
<pinref part="U$1" gate="G$1" pin="UVCC"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VBUS"/>
<pinref part="U$2" gate="G$1" pin="UVCC"/>
</segment>
<segment>
<pinref part="F1" gate="G$1" pin="2"/>
<pinref part="D1" gate="G$1" pin="A"/>
<junction x="-129.54" y="40.64"/>
<pinref part="C13" gate="G$1" pin="2"/>
<wire x1="-129.54" y1="45.72" x2="-129.54" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-132.08" y1="45.72" x2="-129.54" y2="45.72" width="0.1524" layer="91"/>
<pinref part="U$3" gate="G$1" pin="UVCC"/>
<junction x="-129.54" y="45.72"/>
</segment>
</net>
<net name="N$34" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="UCAP"/>
<pinref part="C6" gate="G$1" pin="2"/>
<wire x1="-35.56" y1="0" x2="-40.64" y2="0" width="0.1524" layer="91"/>
<wire x1="-40.64" y1="0" x2="-40.64" y2="-2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$36" class="0">
<segment>
<wire x1="7.62" y1="43.18" x2="-50.8" y2="43.18" width="0.1524" layer="91"/>
<wire x1="-50.8" y1="43.18" x2="-50.8" y2="17.78" width="0.1524" layer="91"/>
<wire x1="-50.8" y1="17.78" x2="-157.48" y2="17.78" width="0.1524" layer="91"/>
<wire x1="-157.48" y1="17.78" x2="-157.48" y2="40.64" width="0.1524" layer="91"/>
<pinref part="R17" gate="G$1" pin="2"/>
<pinref part="R16" gate="G$1" pin="2"/>
<wire x1="-167.64" y1="35.56" x2="-167.64" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-157.48" y1="40.64" x2="-162.56" y2="40.64" width="0.1524" layer="91"/>
<junction x="-167.64" y="40.64"/>
<pinref part="C12" gate="G$1" pin="2"/>
<wire x1="-162.56" y1="40.64" x2="-167.64" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-162.56" y1="35.56" x2="-162.56" y2="40.64" width="0.1524" layer="91"/>
<junction x="-162.56" y="40.64"/>
<pinref part="IC1" gate="G$1" pin="ADC7/PF7"/>
<wire x1="7.62" y1="43.18" x2="7.62" y2="35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$47" class="0">
<segment>
<wire x1="-48.26" y1="15.24" x2="-48.26" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-48.26" y1="40.64" x2="5.08" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-187.96" y1="15.24" x2="-48.26" y2="15.24" width="0.1524" layer="91"/>
<pinref part="IC1" gate="G$1" pin="ADC6/PF6"/>
<wire x1="5.08" y1="40.64" x2="5.08" y2="35.56" width="0.1524" layer="91"/>
<pinref part="R14" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$58" class="0">
<segment>
<pinref part="Q3" gate="G$1" pin="S"/>
<wire x1="-182.88" y1="30.48" x2="-182.88" y2="40.64" width="0.1524" layer="91"/>
<pinref part="R15" gate="G$1" pin="1"/>
<wire x1="-182.88" y1="40.64" x2="-187.96" y2="40.64" width="0.1524" layer="91"/>
<junction x="-187.96" y="40.64"/>
<pinref part="Q2" gate="G$1" pin="S"/>
<wire x1="-193.04" y1="40.64" x2="-187.96" y2="40.64" width="0.1524" layer="91"/>
<wire x1="-193.04" y1="30.48" x2="-193.04" y2="40.64" width="0.1524" layer="91"/>
<pinref part="Q1" gate="G$1" pin="S"/>
<junction x="-193.04" y="40.64"/>
</segment>
</net>
<net name="N$59" class="0">
<segment>
<pinref part="Q3" gate="G$1" pin="D"/>
<pinref part="R16" gate="G$1" pin="1"/>
<wire x1="-182.88" y1="20.32" x2="-180.34" y2="20.32" width="0.1524" layer="91"/>
<wire x1="-180.34" y1="20.32" x2="-180.34" y2="40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$32" class="0">
<segment>
<pinref part="Q3" gate="G$1" pin="G"/>
<pinref part="R15" gate="G$1" pin="2"/>
<pinref part="R14" gate="G$1" pin="2"/>
<junction x="-187.96" y="27.94"/>
</segment>
</net>
<net name="N$53" class="0">
<segment>
<pinref part="Q2" gate="G$1" pin="G"/>
<pinref part="R12" gate="G$1" pin="2"/>
<wire x1="-198.12" y1="25.4" x2="-198.12" y2="27.94" width="0.1524" layer="91"/>
<pinref part="R13" gate="G$1" pin="2"/>
<wire x1="-200.66" y1="25.4" x2="-200.66" y2="27.94" width="0.1524" layer="91"/>
<wire x1="-200.66" y1="27.94" x2="-198.12" y2="27.94" width="0.1524" layer="91"/>
<junction x="-198.12" y="27.94"/>
</segment>
</net>
<net name="N$55" class="0">
<segment>
<pinref part="Q1" gate="G$1" pin="D"/>
<pinref part="CON3" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$37" class="0">
<segment>
<pinref part="IC3" gate="G$1" pin="BYP"/>
<wire x1="-83.82" y1="40.64" x2="-83.82" y2="48.26" width="0.1524" layer="91"/>
<wire x1="-83.82" y1="48.26" x2="-53.34" y2="48.26" width="0.1524" layer="91"/>
<pinref part="C18" gate="G$1" pin="2"/>
<wire x1="-53.34" y1="35.56" x2="-53.34" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$35" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="1"/>
<pinref part="LED1" gate="G$1" pin="A"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<pinref part="R5" gate="G$1" pin="2"/>
<wire x1="58.42" y1="7.62" x2="58.42" y2="5.08" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="PA10/RX1"/>
<wire x1="58.42" y1="5.08" x2="58.42" y2="2.54" width="0.1524" layer="91"/>
<wire x1="73.66" y1="5.08" x2="58.42" y2="5.08" width="0.1524" layer="91"/>
<junction x="58.42" y="5.08"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="R8" gate="G$1" pin="1"/>
<pinref part="R7" gate="G$1" pin="2"/>
<wire x1="60.96" y1="7.62" x2="60.96" y2="2.54" width="0.1524" layer="91"/>
<pinref part="MOD1" gate="G$1" pin="PA9/TX1"/>
<wire x1="73.66" y1="7.62" x2="60.96" y2="7.62" width="0.1524" layer="91"/>
<junction x="60.96" y="7.62"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="C10" gate="G$1" pin="2"/>
<pinref part="MOD1" gate="G$1" pin="VCC"/>
<wire x1="68.58" y1="-5.08" x2="73.66" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="C9" gate="G$1" pin="2"/>
<junction x="73.66" y="-5.08"/>
<wire x1="68.58" y1="-5.08" x2="63.5" y2="-5.08" width="0.1524" layer="91"/>
<junction x="68.58" y="-5.08"/>
<wire x1="63.5" y1="-5.08" x2="63.5" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="63.5" y1="-30.48" x2="78.74" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="8"/>
<wire x1="78.74" y1="-30.48" x2="78.74" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="78.74" y1="-45.72" x2="73.66" y2="-45.72" width="0.1524" layer="91"/>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="63.5" y1="-5.08" x2="63.5" y2="20.32" width="0.1524" layer="91"/>
<wire x1="63.5" y1="20.32" x2="60.96" y2="20.32" width="0.1524" layer="91"/>
<junction x="63.5" y="-5.08"/>
<pinref part="R6" gate="G$1" pin="2"/>
<wire x1="60.96" y1="20.32" x2="58.42" y2="20.32" width="0.1524" layer="91"/>
<junction x="60.96" y="20.32"/>
<wire x1="78.74" y1="-30.48" x2="132.08" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="132.08" y1="-30.48" x2="132.08" y2="22.86" width="0.1524" layer="91"/>
<junction x="78.74" y="-30.48"/>
<pinref part="R11" gate="G$1" pin="2"/>
<wire x1="132.08" y1="22.86" x2="127" y2="22.86" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="60.96" y1="-10.16" x2="60.96" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-33.02" x2="76.2" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="76.2" y1="-33.02" x2="76.2" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="7"/>
<wire x1="76.2" y1="-43.18" x2="73.66" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$60" class="0">
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="58.42" y1="-10.16" x2="58.42" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="58.42" y1="-35.56" x2="73.66" y2="-35.56" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="6"/>
<wire x1="73.66" y1="-35.56" x2="73.66" y2="-40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$61" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="HWB/PE2"/>
<wire x1="35.56" y1="12.7" x2="35.56" y2="27.94" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="2"/>
<pinref part="R1" gate="G$1" pin="1"/>
<wire x1="35.56" y1="27.94" x2="40.64" y2="27.94" width="0.1524" layer="91"/>
<junction x="40.64" y="27.94"/>
</segment>
</net>
<net name="N$63" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="BOOT0"/>
<wire x1="101.6" y1="-25.4" x2="101.6" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="R9" gate="G$1" pin="1"/>
<wire x1="101.6" y1="-25.4" x2="81.28" y2="-25.4" width="0.1524" layer="91"/>
<wire x1="81.28" y1="-25.4" x2="81.28" y2="-50.8" width="0.1524" layer="91"/>
<junction x="101.6" y="-25.4"/>
<pinref part="CON2" gate="G$1" pin="10"/>
<wire x1="81.28" y1="-50.8" x2="73.66" y2="-50.8" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$64" class="0">
<segment>
<pinref part="SW1" gate="G$1" pin="1"/>
<wire x1="-43.18" y1="-63.5" x2="-43.18" y2="-58.42" width="0.1524" layer="91"/>
<wire x1="-43.18" y1="-58.42" x2="-30.48" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="4"/>
<wire x1="-30.48" y1="-58.42" x2="-30.48" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="R4" gate="G$1" pin="2"/>
<pinref part="C8" gate="G$1" pin="2"/>
<wire x1="-20.32" y1="-48.26" x2="-25.4" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="-48.26" x2="-30.48" y2="-48.26" width="0.1524" layer="91"/>
<junction x="-25.4" y="-48.26"/>
<pinref part="IC2" gate="G$1" pin="OUT"/>
<pinref part="IC1" gate="G$1" pin="!RESET"/>
<wire x1="-30.48" y1="-48.26" x2="-35.56" y2="-48.26" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="-35.56" x2="-10.16" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="-10.16" y1="-45.72" x2="-20.32" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-45.72" x2="-20.32" y2="-48.26" width="0.1524" layer="91"/>
<junction x="-20.32" y="-48.26"/>
<pinref part="CON1" gate="G$1" pin="5"/>
<wire x1="-58.42" y1="-22.86" x2="-58.42" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-58.42" y1="-27.94" x2="-35.56" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-35.56" y1="-27.94" x2="-35.56" y2="-48.26" width="0.1524" layer="91"/>
<junction x="-35.56" y="-48.26"/>
<wire x1="-30.48" y1="-58.42" x2="-30.48" y2="-48.26" width="0.1524" layer="91"/>
<junction x="-30.48" y="-58.42"/>
<junction x="-30.48" y="-48.26"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD1/SDA/INT1"/>
<wire x1="5.08" y1="-35.56" x2="5.08" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="5.08" y1="-43.18" x2="-33.02" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-43.18" x2="-33.02" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-33.02" x2="-91.44" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="CON5" gate="G$1" pin="4"/>
<wire x1="-91.44" y1="-33.02" x2="-91.44" y2="-35.56" width="0.1524" layer="91"/>
<pinref part="R20" gate="G$1" pin="1"/>
<wire x1="-91.44" y1="-35.56" x2="-91.44" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="-86.36" y1="-35.56" x2="-91.44" y2="-35.56" width="0.1524" layer="91"/>
<junction x="-91.44" y="-35.56"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC6"/>
<wire x1="35.56" y1="7.62" x2="53.34" y2="7.62" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="1"/>
<wire x1="53.34" y1="7.62" x2="53.34" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="53.34" y1="-40.64" x2="55.88" y2="-40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PC7"/>
<wire x1="35.56" y1="10.16" x2="50.8" y2="10.16" width="0.1524" layer="91"/>
<wire x1="50.8" y1="10.16" x2="50.8" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="CON2" gate="G$1" pin="2"/>
<wire x1="50.8" y1="-43.18" x2="55.88" y2="-43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PCINT6/ADC13/PB6"/>
<pinref part="LED1" gate="G$1" pin="K"/>
<wire x1="35.56" y1="5.08" x2="45.72" y2="5.08" width="0.1524" layer="91"/>
<wire x1="45.72" y1="5.08" x2="45.72" y2="15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD3/TXD1/INT3"/>
<wire x1="10.16" y1="-35.56" x2="10.16" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="10.16" y1="-40.64" x2="20.32" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="20.32" y1="-40.64" x2="20.32" y2="-76.2" width="0.1524" layer="91"/>
<wire x1="20.32" y1="-76.2" x2="-86.36" y2="-76.2" width="0.1524" layer="91"/>
<pinref part="R23" gate="G$1" pin="1"/>
<wire x1="-86.36" y1="-43.18" x2="-86.36" y2="-63.5" width="0.1524" layer="91"/>
<pinref part="CON5" gate="G$1" pin="1"/>
<wire x1="-86.36" y1="-63.5" x2="-86.36" y2="-76.2" width="0.1524" layer="91"/>
<wire x1="-91.44" y1="-60.96" x2="-91.44" y2="-63.5" width="0.1524" layer="91"/>
<wire x1="-91.44" y1="-63.5" x2="-86.36" y2="-63.5" width="0.1524" layer="91"/>
<junction x="-86.36" y="-63.5"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="PD2/RXD1/INT2"/>
<wire x1="7.62" y1="-35.56" x2="7.62" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="7.62" y1="-43.18" x2="17.78" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="17.78" y1="-43.18" x2="17.78" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="17.78" y1="-73.66" x2="-101.6" y2="-73.66" width="0.1524" layer="91"/>
<wire x1="-101.6" y1="-73.66" x2="-101.6" y2="-40.64" width="0.1524" layer="91"/>
<pinref part="CON5" gate="G$1" pin="6"/>
<wire x1="-101.6" y1="-40.64" x2="-96.52" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="-96.52" y1="-40.64" x2="-96.52" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="R22" gate="G$1" pin="1"/>
<wire x1="-86.36" y1="-40.64" x2="-96.52" y2="-40.64" width="0.1524" layer="91"/>
<junction x="-96.52" y="-40.64"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
