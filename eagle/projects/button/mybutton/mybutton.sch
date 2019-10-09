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
<package name="ESP-12S">
<description>&lt;b&gt;ESP-12S&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-8" y="7" dx="2.6" dy="1.1" layer="1"/>
<smd name="2" x="-8" y="5" dx="2.6" dy="1.1" layer="1"/>
<smd name="3" x="-8" y="3" dx="2.6" dy="1.1" layer="1"/>
<smd name="4" x="-8" y="1" dx="2.6" dy="1.1" layer="1"/>
<smd name="5" x="-8" y="-1" dx="2.6" dy="1.1" layer="1"/>
<smd name="6" x="-8" y="-3" dx="2.6" dy="1.1" layer="1"/>
<smd name="7" x="-8" y="-5" dx="2.6" dy="1.1" layer="1"/>
<smd name="8" x="-8" y="-7" dx="2.6" dy="1.1" layer="1"/>
<smd name="9" x="8" y="-7" dx="2.6" dy="1.1" layer="1"/>
<smd name="10" x="8" y="-5" dx="2.6" dy="1.1" layer="1"/>
<smd name="11" x="8" y="-3" dx="2.6" dy="1.1" layer="1"/>
<smd name="12" x="8" y="-1" dx="2.6" dy="1.1" layer="1"/>
<smd name="13" x="8" y="1" dx="2.6" dy="1.1" layer="1"/>
<smd name="14" x="8" y="3" dx="2.6" dy="1.1" layer="1"/>
<smd name="15" x="8" y="5" dx="2.6" dy="1.1" layer="1"/>
<smd name="16" x="8" y="7" dx="2.6" dy="1.1" layer="1"/>
<text x="-4.912" y="16.421" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="2.188" y="16.421" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-8" y1="15.5" x2="8" y2="15.5" width="0.2" layer="21"/>
<wire x1="8" y1="15.5" x2="8" y2="-8.5" width="0.1" layer="51"/>
<wire x1="8" y1="-8.5" x2="-8" y2="-8.5" width="0.2" layer="21"/>
<wire x1="-8" y1="-8.5" x2="-8" y2="15.5" width="0.1" layer="51"/>
<wire x1="-8" y1="-8.5" x2="8" y2="-8.5" width="0.1" layer="51"/>
<wire x1="-8" y1="15.5" x2="8" y2="15.5" width="0.1" layer="51"/>
<wire x1="-8" y1="15.5" x2="-8" y2="7.95" width="0.2" layer="21"/>
<wire x1="8" y1="15.5" x2="8" y2="7.95" width="0.2" layer="21"/>
<wire x1="-2.5" y1="4.6" x2="2.5" y2="4.6" width="0.1" layer="51"/>
<wire x1="2.5" y1="4.6" x2="2.5" y2="-0.4" width="0.1" layer="51"/>
<wire x1="2.5" y1="-0.4" x2="-2.5" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-2.5" y1="-0.4" x2="-2.5" y2="4.6" width="0.1" layer="51"/>
</package>
<package name="LED-RGB-3528-PLCC4">
<description>&lt;b&gt;3.5x2.8mm_Surface_Mount_LED_LAMP&lt;/b&gt;&lt;br&gt;
</description>
<smd name="3" x="1.4" y="-0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="2" x="-1.4" y="-0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="1" x="-1.4" y="0.725" dx="1.2" dy="0.9" layer="1"/>
<smd name="4" x="1.4" y="0.725" dx="1.2" dy="0.9" layer="1"/>
<text x="0.888340625" y="2.6047" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-1.6" y1="1.4" x2="1.6" y2="1.4" width="0.1" layer="51"/>
<wire x1="1.6" y1="1.4" x2="1.6" y2="-1.4" width="0.1" layer="51"/>
<wire x1="1.6" y1="-1.4" x2="-1.6" y2="-1.4" width="0.1" layer="51"/>
<wire x1="-1.6" y1="-1.4" x2="-1.6" y2="1.4" width="0.1" layer="51"/>
<circle x="-0.5" y="0.747" radius="0.071" width="0.2" layer="21"/>
<wire x1="-1.6" y1="1.4" x2="1.6" y2="1.4" width="0.2" layer="21"/>
<wire x1="-1.6" y1="-1.4" x2="1.6" y2="-1.4" width="0.2" layer="21"/>
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
<package name="SOT23-5">
<description>&lt;b&gt;5-Lead SOT-23&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-1.25" y="0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="2" x="-1.25" y="0" dx="1.2" dy="0.6" layer="1"/>
<smd name="3" x="-1.25" y="-0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="4" x="1.25" y="-0.95" dx="1.2" dy="0.6" layer="1"/>
<smd name="5" x="1.25" y="0.95" dx="1.2" dy="0.6" layer="1"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<text x="0" y="1.27" size="1.27" layer="27" align="center">&gt;VALUE</text>
<wire x1="-0.85" y1="1.45" x2="0.85" y2="1.45" width="0.1" layer="51"/>
<wire x1="0.85" y1="1.45" x2="0.85" y2="-1.45" width="0.1" layer="51"/>
<wire x1="0.85" y1="-1.45" x2="-0.85" y2="-1.45" width="0.1" layer="51"/>
<wire x1="-0.85" y1="-1.45" x2="-0.85" y2="1.45" width="0.1" layer="51"/>
<wire x1="-0.85" y1="0.5" x2="0.1" y2="1.45" width="0.1" layer="51"/>
<wire x1="-0.3" y1="1.45" x2="0.3" y2="1.45" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.45" x2="0.3" y2="-1.45" width="0.2" layer="21"/>
<wire x1="0.3" y1="-1.45" x2="-0.3" y2="-1.45" width="0.2" layer="21"/>
<wire x1="-0.3" y1="-1.45" x2="-0.3" y2="1.45" width="0.2" layer="21"/>
<wire x1="-1.85" y1="1.5" x2="-0.65" y2="1.5" width="0.2" layer="21"/>
</package>
<package name="REED-SMD-5MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<text x="-0.03" y="0.04" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-3" y1="1" x2="-3" y2="-1" width="0.1" layer="51"/>
<wire x1="-3" y1="-1" x2="3" y2="-1" width="0.1" layer="51"/>
<wire x1="3" y1="-1" x2="3" y2="1" width="0.1" layer="51"/>
<wire x1="3" y1="1" x2="-3" y2="1" width="0.1" layer="51"/>
<wire x1="-3" y1="1" x2="3" y2="1" width="0.2" layer="21"/>
<wire x1="3" y1="1" x2="3" y2="-1" width="0.2" layer="21"/>
<wire x1="3" y1="-1" x2="-3" y2="-1" width="0.2" layer="21"/>
<wire x1="-3" y1="-1" x2="-3" y2="1" width="0.2" layer="21"/>
<smd name="P$1" x="-4.8" y="0" dx="3" dy="1" layer="1"/>
<smd name="P$2" x="4.8" y="0" dx="3" dy="1" layer="1"/>
</package>
<package name="HEADER-6P-DIP-2_54MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="0" y="0" drill="1.1" diameter="1.65"/>
<pad name="2" x="2.54" y="0" drill="1.1" diameter="1.65"/>
<pad name="3" x="5.08" y="0" drill="1.1" diameter="1.65"/>
<pad name="4" x="7.62" y="0" drill="1.1" diameter="1.65"/>
<pad name="5" x="10.16" y="0" drill="1.1" diameter="1.65"/>
<text x="1.27" y="2.54" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-1.52" y1="1.27" x2="14.22" y2="1.27" width="0.1" layer="51"/>
<wire x1="14.22" y1="1.27" x2="14.22" y2="-1.27" width="0.1" layer="51"/>
<wire x1="14.22" y1="-1.27" x2="-1.52" y2="-1.27" width="0.1" layer="51"/>
<wire x1="-1.52" y1="-1.27" x2="14.22" y2="-1.27" width="0.2" layer="21"/>
<wire x1="14.22" y1="-1.27" x2="14.22" y2="1.27" width="0.2" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="14.22" y2="1.27" width="0.2" layer="21"/>
<wire x1="-1.524" y1="1.27" x2="-1.524" y2="-1.27" width="0.1" layer="51"/>
<wire x1="-1.524" y1="1.27" x2="-1.524" y2="-1.27" width="0.2" layer="21"/>
<circle x="-1.1" y="0.8" radius="0.25" width="0" layer="21"/>
<pad name="6" x="12.7" y="0" drill="1.1" diameter="1.65"/>
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
<package name="BCAAAPC">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="0" y="0" drill="1.1" diameter="2"/>
<pad name="2" x="46.8" y="0" drill="1.1" diameter="2"/>
<text x="1.82" y="7.89" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="4" y1="2" x2="4" y2="-2" width="0.2" layer="21"/>
<wire x1="2" y1="0" x2="6" y2="0" width="0.2" layer="21"/>
<wire x1="45" y1="0" x2="41" y2="0" width="0.2" layer="21"/>
<wire x1="-1.85" y1="6.6" x2="48.65" y2="6.6" width="0.2" layer="21"/>
<wire x1="48.65" y1="6.6" x2="48.65" y2="-6.6" width="0.2" layer="21"/>
<wire x1="48.65" y1="-6.6" x2="-1.85" y2="-6.6" width="0.2" layer="21"/>
<wire x1="-1.85" y1="-6.6" x2="-1.85" y2="6.6" width="0.2" layer="21"/>
<wire x1="-1.85" y1="6.6" x2="48.65" y2="6.6" width="0.1" layer="51"/>
<wire x1="48.65" y1="6.6" x2="48.65" y2="-6.6" width="0.1" layer="51"/>
<wire x1="48.65" y1="-6.6" x2="-1.85" y2="-6.6" width="0.1" layer="51"/>
<wire x1="-1.85" y1="-6.6" x2="-1.85" y2="6.6" width="0.1" layer="51"/>
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
</packages>
<symbols>
<symbol name="ESP-12S">
<wire x1="5.08" y1="2.54" x2="27.94" y2="2.54" width="0.254" layer="94"/>
<wire x1="27.94" y1="-20.32" x2="27.94" y2="2.54" width="0.254" layer="94"/>
<wire x1="27.94" y1="-20.32" x2="5.08" y2="-20.32" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-20.32" width="0.254" layer="94"/>
<text x="6.35" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="6.35" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="RST" x="0" y="0" length="middle"/>
<pin name="ADC" x="0" y="-2.54" length="middle"/>
<pin name="EN" x="0" y="-5.08" length="middle"/>
<pin name="GPIO16" x="0" y="-7.62" length="middle"/>
<pin name="GPIO14" x="0" y="-10.16" length="middle"/>
<pin name="GPIO12" x="0" y="-12.7" length="middle"/>
<pin name="GPIO13" x="0" y="-15.24" length="middle"/>
<pin name="VCC" x="0" y="-17.78" length="middle"/>
<pin name="GND" x="33.02" y="-17.78" length="middle" rot="R180"/>
<pin name="GPIO15" x="33.02" y="-15.24" length="middle" rot="R180"/>
<pin name="GPIO2" x="33.02" y="-12.7" length="middle" rot="R180"/>
<pin name="GPIO0" x="33.02" y="-10.16" length="middle" rot="R180"/>
<pin name="GPIO4" x="33.02" y="-7.62" length="middle" rot="R180"/>
<pin name="GPIO5" x="33.02" y="-5.08" length="middle" rot="R180"/>
<pin name="RXD0" x="33.02" y="-2.54" length="middle" rot="R180"/>
<pin name="TXD0" x="33.02" y="0" length="middle" rot="R180"/>
</symbol>
<symbol name="LED-RGB-A">
<wire x1="5.08" y1="2.54" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="15.24" y2="2.54" width="0.254" layer="94"/>
<wire x1="15.24" y1="-5.08" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-5.08" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="R" x="0" y="0" length="middle"/>
<pin name="G" x="0" y="-2.54" length="middle"/>
<pin name="A" x="20.32" y="0" length="middle" rot="R180"/>
<pin name="B" x="20.32" y="-2.54" length="middle" rot="R180"/>
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
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-3.048" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
<wire x1="-1.27" y1="-0.508" x2="1.27" y2="-0.508" width="0.254" layer="94"/>
<wire x1="-0.508" y1="-1.016" x2="0.508" y2="-1.016" width="0.254" layer="94"/>
</symbol>
<symbol name="MIC5205">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="VIN" x="0" y="0" length="middle" direction="pwr"/>
<pin name="GND" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="EN" x="0" y="-5.08" length="middle" direction="in"/>
<pin name="BYP" x="27.94" y="-5.08" length="middle" direction="out" rot="R180"/>
<pin name="VOUT" x="27.94" y="0" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="REED">
<wire x1="2.54" y1="1.016" x2="10.16" y2="1.016" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.016" x2="10.16" y2="1.016" width="0.254" layer="94"/>
<wire x1="10.16" y1="-1.016" x2="2.54" y2="-1.016" width="0.254" layer="94"/>
<wire x1="2.54" y1="1.016" x2="2.54" y2="-1.016" width="0.254" layer="94"/>
<text x="2.54" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="2.54" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
<wire x1="2.6" y1="0" x2="5.6" y2="0" width="0.1524" layer="94"/>
<wire x1="5.6" y1="0" x2="6.4" y2="0.8" width="0.1524" layer="94"/>
<wire x1="6.4" y1="0" x2="10" y2="0" width="0.1524" layer="94"/>
</symbol>
<symbol name="VCC">
<circle x="0" y="0.762" radius="0.762" width="0.254" layer="94"/>
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="CONNECTOR-6P">
<wire x1="5.08" y1="2.54" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-15.24" x2="12.7" y2="2.54" width="0.254" layer="94"/>
<wire x1="12.7" y1="-15.24" x2="5.08" y2="-15.24" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-15.24" width="0.254" layer="94"/>
<text x="5.08" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.08" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" length="middle"/>
<pin name="2" x="0" y="-2.54" length="middle"/>
<pin name="3" x="0" y="-5.08" length="middle"/>
<pin name="4" x="0" y="-7.62" length="middle"/>
<pin name="5" x="0" y="-10.16" length="middle"/>
<pin name="6" x="0" y="-12.7" length="middle"/>
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
<symbol name="BATTERY">
<wire x1="5.588" y1="2.032" x2="5.588" y2="-2.032" width="0.254" layer="94"/>
<wire x1="5.08" y1="0" x2="5.588" y2="0" width="0.254" layer="94"/>
<wire x1="7.112" y1="0" x2="7.62" y2="0" width="0.254" layer="94"/>
<text x="3.81" y="3.048" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="3.81" y="4.826" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="+" x="0" y="0" visible="off" length="middle" direction="pwr"/>
<pin name="-" x="12.7" y="0" visible="off" length="middle" direction="pwr" rot="R180"/>
<wire x1="7.112" y1="1.016" x2="7.112" y2="-1.016" width="0.254" layer="94"/>
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
</symbols>
<devicesets>
<deviceset name="ESP-12S" prefix="MOD" uservalue="yes">
<gates>
<gate name="G$1" symbol="ESP-12S" x="0" y="0"/>
</gates>
<devices>
<device name="" package="ESP-12S">
<connects>
<connect gate="G$1" pin="ADC" pad="2"/>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND" pad="9"/>
<connect gate="G$1" pin="GPIO0" pad="12"/>
<connect gate="G$1" pin="GPIO12" pad="6"/>
<connect gate="G$1" pin="GPIO13" pad="7"/>
<connect gate="G$1" pin="GPIO14" pad="5"/>
<connect gate="G$1" pin="GPIO15" pad="10"/>
<connect gate="G$1" pin="GPIO16" pad="4"/>
<connect gate="G$1" pin="GPIO2" pad="11"/>
<connect gate="G$1" pin="GPIO4" pad="13"/>
<connect gate="G$1" pin="GPIO5" pad="14"/>
<connect gate="G$1" pin="RST" pad="1"/>
<connect gate="G$1" pin="RXD0" pad="15"/>
<connect gate="G$1" pin="TXD0" pad="16"/>
<connect gate="G$1" pin="VCC" pad="8"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="LED-RGB-3528-PLCC4" prefix="LED" uservalue="yes">
<gates>
<gate name="G$1" symbol="LED-RGB-A" x="0" y="0"/>
</gates>
<devices>
<device name="" package="LED-RGB-3528-PLCC4">
<connects>
<connect gate="G$1" pin="A" pad="4"/>
<connect gate="G$1" pin="B" pad="3"/>
<connect gate="G$1" pin="G" pad="2"/>
<connect gate="G$1" pin="R" pad="1"/>
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
<deviceset name="MIC5205" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="MIC5205" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-5">
<connects>
<connect gate="G$1" pin="BYP" pad="4"/>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="VIN" pad="1"/>
<connect gate="G$1" pin="VOUT" pad="5"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="REED-DIP-5MM" prefix="RD" uservalue="yes">
<gates>
<gate name="G$1" symbol="REED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="REED-SMD-5MM">
<connects>
<connect gate="G$1" pin="1" pad="P$1"/>
<connect gate="G$1" pin="2" pad="P$2"/>
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
<deviceset name="HEADER-6P-DIP-2_54MM" prefix="CON" uservalue="yes">
<gates>
<gate name="G$1" symbol="CONNECTOR-6P" x="0" y="0"/>
</gates>
<devices>
<device name="" package="HEADER-6P-DIP-2_54MM">
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
<deviceset name="BCAAAPC" prefix="BAT" uservalue="yes">
<gates>
<gate name="G$1" symbol="BATTERY" x="0" y="0"/>
</gates>
<devices>
<device name="" package="BCAAAPC">
<connects>
<connect gate="G$1" pin="+" pad="1"/>
<connect gate="G$1" pin="-" pad="2"/>
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
<deviceset name="DIODE-0603" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="DIODE" x="0" y="0"/>
</gates>
<devices>
<device name="0603" package="0603">
<connects>
<connect gate="G$1" pin="A" pad="1"/>
<connect gate="G$1" pin="K" pad="2"/>
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
<class number="0" name="default" width="0.381" drill="0.381">
<clearance class="0" value="0.254"/>
</class>
</classes>
<parts>
<part name="MOD1" library="SamacSys_Parts" deviceset="ESP-12S" device=""/>
<part name="LED1" library="SamacSys_Parts" deviceset="LED-RGB-3528-PLCC4" device=""/>
<part name="SW1" library="SamacSys_Parts" deviceset="BUTTON-4P-SMD-5X5" device=""/>
<part name="IC1" library="SamacSys_Parts" deviceset="MIC5205" device=""/>
<part name="RD1" library="SamacSys_Parts" deviceset="REED-DIP-5MM" device=""/>
<part name="CON1" library="SamacSys_Parts" deviceset="HEADER-6P-DIP-2_54MM" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R2" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R3" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R4" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R5" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R6" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP4" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="BAT1" library="SamacSys_Parts" deviceset="BCAAAPC" device=""/>
<part name="Q1" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="R12" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="R11" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R10" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="D1" library="SamacSys_Parts" deviceset="DIODE-0603" device="0603"/>
<part name="R13" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C3" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C4" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="R8" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R14" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND6" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND7" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C5" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND9" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C6" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND10" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP5" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R7" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP7" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND11" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R9" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP8" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP3" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP6" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND8" library="SamacSys_Parts" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="MOD1" gate="G$1" x="0" y="0" smashed="yes">
<attribute name="NAME" x="5.334" y="3.556" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="6.35" y="7.62" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="LED1" gate="G$1" x="-25.4" y="-30.48" smashed="yes">
<attribute name="NAME" x="-19.812" y="-26.924" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-19.05" y="-24.13" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SW1" gate="G$1" x="25.4" y="-30.48" smashed="yes">
<attribute name="NAME" x="28.194" y="-26.924" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="29.21" y="-24.13" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="IC1" gate="G$1" x="48.26" y="-38.1" smashed="yes">
<attribute name="NAME" x="53.848" y="-34.544" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="72.39" y="-33.02" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="RD1" gate="G$1" x="10.16" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="8.128" y="-41.402" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="5.08" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="CON1" gate="G$1" x="68.58" y="0" smashed="yes">
<attribute name="NAME" x="73.66" y="3.81" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="73.66" y="6.35" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND2" gate="1" x="33.02" y="-20.32" smashed="yes">
<attribute name="VALUE" x="30.48" y="-22.86" size="1.778" layer="96"/>
</instance>
<instance part="C1" gate="G$1" x="-12.7" y="5.08" smashed="yes">
<attribute name="NAME" x="-10.922" y="6.096" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="10.16" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R1" gate="G$1" x="-12.7" y="0" smashed="yes">
<attribute name="NAME" x="-13.462" y="1.016" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="4.064" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R2" gate="G$1" x="-12.7" y="-2.54" smashed="yes">
<attribute name="NAME" x="-13.462" y="-1.524" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="1.524" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R3" gate="G$1" x="-12.7" y="-5.08" smashed="yes">
<attribute name="NAME" x="-13.462" y="-4.064" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="-1.016" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP1" gate="G$1" x="-15.24" y="-5.08" smashed="yes" rot="R90">
<attribute name="VALUE" x="-17.145" y="-6.985" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP2" gate="G$1" x="-15.24" y="0" smashed="yes" rot="R90">
<attribute name="VALUE" x="-17.145" y="-1.905" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND1" gate="1" x="-15.24" y="5.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="-18.288" y="7.62" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R4" gate="G$1" x="-12.7" y="-10.16" smashed="yes">
<attribute name="NAME" x="-13.462" y="-9.144" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="-6.096" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R5" gate="G$1" x="-12.7" y="-12.7" smashed="yes">
<attribute name="NAME" x="-13.462" y="-11.684" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="-8.636" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R6" gate="G$1" x="-12.7" y="-15.24" smashed="yes">
<attribute name="NAME" x="-13.462" y="-14.224" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="-11.176" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP4" gate="G$1" x="-5.08" y="-27.94" smashed="yes">
<attribute name="VALUE" x="-6.985" y="-26.035" size="1.778" layer="96"/>
</instance>
<instance part="BAT1" gate="G$1" x="0" y="-45.72" smashed="yes" rot="R270">
<attribute name="NAME" x="-2.794" y="-49.022" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="5.08" y="-49.784" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="Q1" gate="G$1" x="12.7" y="-55.88" smashed="yes">
<attribute name="NAME" x="11.938" y="-53.34" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="19.304" y="-54.102" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R12" gate="G$1" x="10.16" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="8.128" y="-54.61" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="6.096" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND3" gate="1" x="10.16" y="-60.96" smashed="yes">
<attribute name="VALUE" x="7.62" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="C2" gate="G$1" x="5.08" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="4.064" y="-50.8" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="0" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R11" gate="G$1" x="35.56" y="-5.08" smashed="yes">
<attribute name="NAME" x="33.528" y="-4.064" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="39.37" y="-1.016" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R10" gate="G$1" x="35.56" y="-7.62" smashed="yes">
<attribute name="NAME" x="33.528" y="-6.604" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="39.37" y="-3.556" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="D1" gate="G$1" x="40.64" y="-43.18" smashed="yes" rot="R180">
<attribute name="NAME" x="35.56" y="-40.894" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="39.116" y="-46.99" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="R13" gate="G$1" x="22.86" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="20.828" y="-54.61" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="18.796" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C3" gate="G$1" x="27.94" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="24.892" y="-53.34" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="22.86" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND4" gate="1" x="0" y="-60.96" smashed="yes">
<attribute name="VALUE" x="-2.54" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="GND5" gate="1" x="5.08" y="-60.96" smashed="yes">
<attribute name="VALUE" x="2.54" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="C4" gate="G$1" x="48.26" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="45.212" y="-53.594" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="43.18" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R8" gate="G$1" x="35.56" y="-12.7" smashed="yes">
<attribute name="NAME" x="34.798" y="-11.684" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="39.37" y="-8.636" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R14" gate="G$1" x="43.18" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="41.148" y="-54.61" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="39.116" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND6" gate="1" x="43.18" y="-60.96" smashed="yes">
<attribute name="VALUE" x="40.64" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="GND7" gate="1" x="48.26" y="-60.96" smashed="yes">
<attribute name="VALUE" x="45.72" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="C5" gate="G$1" x="76.2" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="75.184" y="-50.8" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="71.12" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND9" gate="1" x="76.2" y="-60.96" smashed="yes">
<attribute name="VALUE" x="73.66" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="C6" gate="G$1" x="81.28" y="-58.42" smashed="yes" rot="R90">
<attribute name="NAME" x="80.264" y="-50.8" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="76.2" y="-54.61" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND10" gate="1" x="81.28" y="-60.96" smashed="yes">
<attribute name="VALUE" x="78.74" y="-64.008" size="1.778" layer="96"/>
</instance>
<instance part="SUP5" gate="G$1" x="81.28" y="-35.56" smashed="yes">
<attribute name="VALUE" x="79.375" y="-33.655" size="1.778" layer="96"/>
</instance>
<instance part="R7" gate="G$1" x="35.56" y="-15.24" smashed="yes">
<attribute name="NAME" x="34.798" y="-14.224" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="39.37" y="-11.176" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP7" gate="G$1" x="66.04" y="-10.16" smashed="yes" rot="R90">
<attribute name="VALUE" x="64.135" y="-12.065" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="GND11" gate="1" x="68.58" y="-15.24" smashed="yes">
<attribute name="VALUE" x="66.04" y="-18.288" size="1.778" layer="96"/>
</instance>
<instance part="R9" gate="G$1" x="60.96" y="-22.86" smashed="yes" rot="R90">
<attribute name="NAME" x="58.928" y="-18.034" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="56.896" y="-19.05" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="SUP8" gate="G$1" x="60.96" y="-25.4" smashed="yes" rot="R180">
<attribute name="VALUE" x="62.865" y="-27.305" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="SUP3" gate="G$1" x="-2.54" y="-17.78" smashed="yes" rot="R90">
<attribute name="VALUE" x="-4.445" y="-19.685" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP6" gate="G$1" x="55.88" y="-15.24" smashed="yes" rot="R180">
<attribute name="VALUE" x="57.785" y="-17.145" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND8" gate="1" x="45.72" y="-40.64" smashed="yes" rot="R270">
<attribute name="VALUE" x="42.672" y="-38.1" size="1.778" layer="96" rot="R270"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$1" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO0"/>
<wire x1="33.02" y1="-10.16" x2="60.96" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="60.96" y1="-10.16" x2="60.96" y2="0" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="1"/>
<wire x1="60.96" y1="0" x2="68.58" y2="0" width="0.1524" layer="91"/>
<pinref part="R9" gate="G$1" pin="2"/>
<junction x="60.96" y="-10.16"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RXD0"/>
<pinref part="CON1" gate="G$1" pin="2"/>
<wire x1="33.02" y1="-2.54" x2="68.58" y2="-2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="TXD0"/>
<wire x1="33.02" y1="0" x2="58.42" y2="0" width="0.1524" layer="91"/>
<wire x1="58.42" y1="0" x2="58.42" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="3"/>
<wire x1="58.42" y1="-5.08" x2="68.58" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RST"/>
<wire x1="0" y1="0" x2="0" y2="5.08" width="0.1524" layer="91"/>
<wire x1="0" y1="5.08" x2="55.88" y2="5.08" width="0.1524" layer="91"/>
<wire x1="55.88" y1="5.08" x2="55.88" y2="-7.62" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="4"/>
<wire x1="55.88" y1="-7.62" x2="68.58" y2="-7.62" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="2"/>
<junction x="0" y="5.08"/>
<pinref part="R1" gate="G$1" pin="2"/>
<junction x="0" y="0"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="R3" gate="G$1" pin="1"/>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="LED1" gate="G$1" pin="A"/>
<pinref part="SUP4" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VOUT"/>
<pinref part="C6" gate="G$1" pin="2"/>
<wire x1="76.2" y1="-38.1" x2="81.28" y2="-38.1" width="0.1524" layer="91"/>
<wire x1="81.28" y1="-38.1" x2="81.28" y2="-45.72" width="0.1524" layer="91"/>
<pinref part="SUP5" gate="G$1" pin="VCC"/>
<junction x="81.28" y="-38.1"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="5"/>
<pinref part="SUP7" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R9" gate="G$1" pin="1"/>
<pinref part="SUP8" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="VCC"/>
<pinref part="SUP3" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R8" gate="G$1" pin="2"/>
<wire x1="48.26" y1="-12.7" x2="55.88" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="SUP6" gate="G$1" pin="VCC"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GND"/>
<pinref part="GND2" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="1"/>
<pinref part="GND1" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R12" gate="G$1" pin="1"/>
<pinref part="GND3" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="BAT1" gate="G$1" pin="-"/>
<pinref part="GND4" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="1"/>
<pinref part="GND5" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R14" gate="G$1" pin="1"/>
<pinref part="GND6" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C4" gate="G$1" pin="1"/>
<pinref part="GND7" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C5" gate="G$1" pin="1"/>
<pinref part="GND9" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="C6" gate="G$1" pin="1"/>
<pinref part="GND10" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="6"/>
<pinref part="GND11" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<pinref part="GND8" gate="1" pin="GND"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="ADC"/>
<pinref part="R2" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="EN"/>
<pinref part="R3" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO14"/>
<pinref part="R4" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO12"/>
<pinref part="R5" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO13"/>
<pinref part="R6" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="-25.4" y1="-12.7" x2="-12.7" y2="-12.7" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="R"/>
<wire x1="-25.4" y1="-12.7" x2="-25.4" y2="-30.48" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<wire x1="-12.7" y1="-15.24" x2="-27.94" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="G"/>
<wire x1="-27.94" y1="-33.02" x2="-25.4" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-27.94" y1="-15.24" x2="-27.94" y2="-33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="R4" gate="G$1" pin="1"/>
<wire x1="-12.7" y1="-10.16" x2="-20.32" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-22.86" x2="-20.32" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="-22.86" x2="-2.54" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="-2.54" y1="-22.86" x2="-2.54" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="B"/>
<wire x1="-2.54" y1="-33.02" x2="-5.08" y2="-33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO5"/>
<pinref part="R11" gate="G$1" pin="1"/>
<wire x1="33.02" y1="-5.08" x2="35.56" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO4"/>
<pinref part="R10" gate="G$1" pin="1"/>
<wire x1="33.02" y1="-7.62" x2="35.56" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$17" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO2"/>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="33.02" y1="-12.7" x2="35.56" y2="-12.7" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$22" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="BYP"/>
<pinref part="C5" gate="G$1" pin="2"/>
<wire x1="76.2" y1="-43.18" x2="76.2" y2="-45.72" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<pinref part="C3" gate="G$1" pin="1"/>
<pinref part="R13" gate="G$1" pin="1"/>
<wire x1="22.86" y1="-58.42" x2="27.94" y2="-58.42" width="0.1524" layer="91"/>
<pinref part="Q1" gate="G$1" pin="D"/>
<wire x1="17.78" y1="-58.42" x2="22.86" y2="-58.42" width="0.1524" layer="91"/>
<junction x="22.86" y="-58.42"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="R7" gate="G$1" pin="2"/>
<wire x1="40.64" y1="-43.18" x2="40.64" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="D1" gate="G$1" pin="K"/>
<wire x1="48.26" y1="-15.24" x2="48.26" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="48.26" y1="-27.94" x2="40.64" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="C4" gate="G$1" pin="2"/>
<pinref part="IC1" gate="G$1" pin="EN"/>
<wire x1="48.26" y1="-43.18" x2="48.26" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="40.64" y1="-43.18" x2="43.18" y2="-43.18" width="0.1524" layer="91"/>
<junction x="40.64" y="-43.18"/>
<junction x="48.26" y="-43.18"/>
<pinref part="R14" gate="G$1" pin="2"/>
<wire x1="43.18" y1="-43.18" x2="48.26" y2="-43.18" width="0.1524" layer="91"/>
<wire x1="43.18" y1="-45.72" x2="43.18" y2="-43.18" width="0.1524" layer="91"/>
<junction x="43.18" y="-43.18"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO15"/>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="33.02" y1="-15.24" x2="35.56" y2="-15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="R11" gate="G$1" pin="2"/>
<wire x1="48.26" y1="-5.08" x2="53.34" y2="-5.08" width="0.1524" layer="91"/>
<wire x1="53.34" y1="-5.08" x2="53.34" y2="-25.4" width="0.1524" layer="91"/>
<wire x1="53.34" y1="-25.4" x2="22.86" y2="-25.4" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="2"/>
<wire x1="25.4" y1="-33.02" x2="22.86" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="22.86" y1="-33.02" x2="22.86" y2="-25.4" width="0.1524" layer="91"/>
<pinref part="D1" gate="G$1" pin="A"/>
<wire x1="22.86" y1="-33.02" x2="22.86" y2="-43.18" width="0.1524" layer="91"/>
<junction x="22.86" y="-33.02"/>
<wire x1="22.86" y1="-43.18" x2="27.94" y2="-43.18" width="0.1524" layer="91"/>
<pinref part="R13" gate="G$1" pin="2"/>
<wire x1="22.86" y1="-43.18" x2="22.86" y2="-45.72" width="0.1524" layer="91"/>
<junction x="22.86" y="-43.18"/>
<pinref part="C3" gate="G$1" pin="2"/>
<wire x1="27.94" y1="-45.72" x2="27.94" y2="-43.18" width="0.1524" layer="91"/>
<junction x="27.94" y="-43.18"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="Q1" gate="G$1" pin="G"/>
<wire x1="12.7" y1="-45.72" x2="12.7" y2="-50.8" width="0.1524" layer="91"/>
<pinref part="RD1" gate="G$1" pin="1"/>
<pinref part="R12" gate="G$1" pin="2"/>
<junction x="10.16" y="-45.72"/>
<wire x1="10.16" y1="-45.72" x2="12.7" y2="-45.72" width="0.1524" layer="91"/>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="48.26" y1="-7.62" x2="50.8" y2="-7.62" width="0.1524" layer="91"/>
<wire x1="50.8" y1="-7.62" x2="50.8" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="50.8" y1="-22.86" x2="12.7" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="12.7" y1="-45.72" x2="12.7" y2="-22.86" width="0.1524" layer="91"/>
<junction x="12.7" y="-45.72"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<wire x1="-17.78" y1="-2.54" x2="-17.78" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="-12.7" y1="-2.54" x2="-17.78" y2="-2.54" width="0.1524" layer="91"/>
<wire x1="-17.78" y1="-20.32" x2="0" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="RD1" gate="G$1" pin="2"/>
<wire x1="0" y1="-30.48" x2="5.08" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="5.08" y1="-30.48" x2="10.16" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="10.16" y1="-30.48" x2="10.16" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="5.08" y1="-45.72" x2="5.08" y2="-30.48" width="0.1524" layer="91"/>
<junction x="5.08" y="-30.48"/>
<wire x1="17.78" y1="-38.1" x2="17.78" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="17.78" y1="-30.48" x2="10.16" y2="-30.48" width="0.1524" layer="91"/>
<junction x="10.16" y="-30.48"/>
<pinref part="SW1" gate="G$1" pin="1"/>
<wire x1="17.78" y1="-30.48" x2="25.4" y2="-30.48" width="0.1524" layer="91"/>
<junction x="17.78" y="-30.48"/>
<pinref part="IC1" gate="G$1" pin="VIN"/>
<wire x1="48.26" y1="-38.1" x2="17.78" y2="-38.1" width="0.1524" layer="91"/>
<wire x1="17.78" y1="-38.1" x2="17.78" y2="-48.26" width="0.1524" layer="91"/>
<junction x="17.78" y="-38.1"/>
<pinref part="Q1" gate="G$1" pin="S"/>
<pinref part="BAT1" gate="G$1" pin="+"/>
<wire x1="0" y1="-45.72" x2="0" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="0" y1="-20.32" x2="0" y2="-30.48" width="0.1524" layer="91"/>
<junction x="0" y="-30.48"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
