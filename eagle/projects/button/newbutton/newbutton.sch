<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="9.3.0">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.01" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
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
<smd name="6" x="1.25" y="0.95" dx="1.27" dy="0.635" layer="1"/>
<wire x1="-0.3" y1="1.05" x2="-0.3" y2="1.25" width="0.2" layer="21"/>
<wire x1="-0.3" y1="1.25" x2="-0.3" y2="1.45" width="0.2" layer="21"/>
<wire x1="-0.3" y1="1.25" x2="0.3" y2="1.25" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.25" x2="0.3" y2="1.05" width="0.2" layer="21"/>
<wire x1="0.3" y1="1.05" x2="-0.3" y2="1.05" width="0.2" layer="21"/>
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
<package name="INDUCTOR-0520-0630">
<description>&lt;b&gt;ERA3A&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-3.1" y="0" dx="3.4" dy="3" layer="1" rot="R90"/>
<smd name="2" x="3.1" y="0" dx="3.4" dy="3" layer="1" rot="R90"/>
<text x="-0.1" y="2.57" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-3.4" y1="3.4" x2="3.4" y2="3.4" width="0.1" layer="51"/>
<wire x1="3.4" y1="3.4" x2="3.4" y2="-3.4" width="0.1" layer="51"/>
<wire x1="3.4" y1="-3.4" x2="-3.4" y2="-3.4" width="0.1" layer="51"/>
<wire x1="-3.4" y1="-3.4" x2="-3.4" y2="3.4" width="0.1" layer="51"/>
<wire x1="-1.6" y1="3.35" x2="1.6" y2="3.35" width="0.2" layer="21"/>
<wire x1="1.6" y1="3.35" x2="1.6" y2="-3.35" width="0.2" layer="21"/>
<wire x1="1.6" y1="-3.35" x2="-1.6" y2="-3.35" width="0.2" layer="21"/>
<wire x1="-1.6" y1="-3.35" x2="-1.6" y2="3.35" width="0.2" layer="21"/>
</package>
<package name="REED-DIP">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<wire x1="-7" y1="1" x2="-7" y2="-1" width="0.1" layer="51"/>
<wire x1="-7" y1="-1" x2="7" y2="-1" width="0.1" layer="51"/>
<wire x1="7" y1="-1" x2="7" y2="1" width="0.1" layer="51"/>
<wire x1="7" y1="1" x2="-7" y2="1" width="0.1" layer="51"/>
<wire x1="-7" y1="1" x2="7" y2="1" width="0.2" layer="21"/>
<wire x1="7" y1="1" x2="7" y2="-1" width="0.2" layer="21"/>
<wire x1="7" y1="-1" x2="-7" y2="-1" width="0.2" layer="21"/>
<wire x1="-7" y1="-1" x2="-7" y2="1" width="0.2" layer="21"/>
<pad name="P$1" x="-10.16" y="0" drill="0.6"/>
<pad name="P$2" x="10.16" y="0" drill="0.6"/>
<text x="-6.85" y="1.2" size="1.27" layer="25">&gt;NAME</text>
<text x="-6.85" y="2.5" size="1.27" layer="27">&gt;VALUE</text>
</package>
<package name="0603-DIODE">
<description>&lt;b&gt;ERA3A&lt;/b&gt;&lt;br&gt;
</description>
<smd name="1" x="-0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<smd name="2" x="0.8" y="0" dx="1" dy="0.95" layer="1" rot="R90"/>
<text x="0" y="1.27" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-0.8" y1="0.4" x2="0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="0.4" x2="0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="0.8" y1="-0.4" x2="-0.8" y2="-0.4" width="0.1" layer="51"/>
<wire x1="-0.8" y1="-0.4" x2="-0.8" y2="0.4" width="0.1" layer="51"/>
<wire x1="-0.15" y1="-0.4" x2="-0.15" y2="0.4" width="0.2" layer="21"/>
<wire x1="-0.15" y1="0.4" x2="0.15" y2="0" width="0.2" layer="21"/>
<wire x1="0.15" y1="0" x2="-0.15" y2="-0.4" width="0.2" layer="21"/>
</package>
<package name="AAA-CONTACTS">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="1" x="-13.95" y="0" drill="1.7"/>
<pad name="2" x="13.95" y="0" drill="1.7"/>
<text x="-0.03" y="4.24" size="1.27" layer="25" align="center">&gt;NAME</text>
<wire x1="-9.45" y1="2" x2="-9.45" y2="-2" width="0.2" layer="21"/>
<wire x1="-11.45" y1="0" x2="-7.45" y2="0" width="0.2" layer="21"/>
<wire x1="11.4" y1="0" x2="7.4" y2="0" width="0.2" layer="21"/>
<wire x1="-22.25" y1="5.3" x2="22.25" y2="5.3" width="0.2" layer="21"/>
<wire x1="22.25" y1="5.3" x2="22.25" y2="-5.3" width="0.2" layer="21"/>
<wire x1="22.25" y1="-5.3" x2="-22.25" y2="-5.3" width="0.2" layer="21"/>
<wire x1="-22.25" y1="-5.3" x2="-22.25" y2="5.3" width="0.2" layer="21"/>
<wire x1="-22.25" y1="5.3" x2="22.25" y2="5.3" width="0.1" layer="51"/>
<wire x1="22.25" y1="5.3" x2="22.25" y2="-5.3" width="0.1" layer="51"/>
<wire x1="22.25" y1="-5.3" x2="-22.25" y2="-5.3" width="0.1" layer="51"/>
<wire x1="-22.25" y1="-5.3" x2="-22.25" y2="5.3" width="0.1" layer="51"/>
<pad name="3" x="-20" y="0" drill="1.7"/>
<pad name="4" x="20" y="0" drill="1.7"/>
</package>
<package name="HEADER-6P-2R-DIP-2_54MM">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<pad name="4" x="0" y="0" drill="1.1" diameter="1.65"/>
<pad name="5" x="2.54" y="0" drill="1.1" diameter="1.65"/>
<pad name="6" x="5.08" y="0" drill="1.1" diameter="1.65"/>
<text x="1.27" y="2.032" size="1.27" layer="25" align="center">&gt;NAME</text>
<pad name="1" x="0" y="-2.54" drill="1.1" diameter="1.65"/>
<pad name="2" x="2.54" y="-2.54" drill="1.1" diameter="1.65"/>
<pad name="3" x="5.08" y="-2.54" drill="1.1" diameter="1.65"/>
<text x="-1.778" y="2.667" size="1.27" layer="51">&gt;VALUE</text>
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
<symbol name="VCC">
<circle x="0" y="0.762" radius="0.762" width="0.254" layer="94"/>
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="MCP1640">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="SW" x="0" y="0" length="middle" direction="pwr"/>
<pin name="GND" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="EN" x="0" y="-5.08" length="middle"/>
<pin name="VFB" x="27.94" y="-5.08" length="middle" rot="R180"/>
<pin name="VOUT" x="27.94" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="VIN" x="27.94" y="0" length="middle" direction="pwr" rot="R180"/>
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
<symbol name="INDUCTOR">
<wire x1="2.54" y1="0" x2="4.445" y2="0" width="0.254" layer="94" curve="-175.4"/>
<wire x1="4.445" y1="0" x2="6.35" y2="0" width="0.254" layer="94" curve="-175.4"/>
<wire x1="6.35" y1="0" x2="8.255" y2="0" width="0.254" layer="94" curve="-175.4"/>
<wire x1="8.255" y1="0" x2="10.16" y2="0" width="0.254" layer="94" curve="-175.4"/>
<text x="2.032" y="2.032" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="2.032" y="3.81" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="1" x="0" y="0" visible="off" length="short" direction="pas"/>
<pin name="2" x="12.7" y="0" visible="off" length="short" direction="pas" rot="R180"/>
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
<deviceset name="MCP1640" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="MCP1640" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-6">
<connects>
<connect gate="G$1" pin="EN" pad="3"/>
<connect gate="G$1" pin="GND" pad="2"/>
<connect gate="G$1" pin="SW" pad="1"/>
<connect gate="G$1" pin="VFB" pad="4"/>
<connect gate="G$1" pin="VIN" pad="6"/>
<connect gate="G$1" pin="VOUT" pad="5"/>
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
<deviceset name="INDUCTOR-0520-0630" prefix="L" uservalue="yes">
<gates>
<gate name="G$1" symbol="INDUCTOR" x="0" y="0"/>
</gates>
<devices>
<device name="" package="INDUCTOR-0520-0630">
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
<deviceset name="REED-DIP" prefix="RD" uservalue="yes">
<gates>
<gate name="G$1" symbol="REED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="REED-DIP">
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
<deviceset name="DIODE-0603" prefix="D" uservalue="yes">
<gates>
<gate name="G$1" symbol="DIODE" x="0" y="0"/>
</gates>
<devices>
<device name="" package="0603-DIODE">
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
<deviceset name="AAA-CONTACTS" prefix="BAT" uservalue="yes">
<gates>
<gate name="G$1" symbol="BATTERY" x="0" y="0"/>
</gates>
<devices>
<device name="" package="AAA-CONTACTS">
<connects>
<connect gate="G$1" pin="+" pad="1 3"/>
<connect gate="G$1" pin="-" pad="2 4"/>
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
<class number="0" name="default" width="0.381" drill="0.381">
<clearance class="0" value="0.254"/>
</class>
</classes>
<parts>
<part name="MOD1" library="SamacSys_Parts" deviceset="ESP-12S" device=""/>
<part name="SW1" library="SamacSys_Parts" deviceset="BUTTON-4P-SMD-5X5" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="LED1" library="SamacSys_Parts" deviceset="LED-RGB-3528-PLCC4" device=""/>
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R2" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R3" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R4" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R5" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R6" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R7" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R8" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R9" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R10" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R12" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP3" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP5" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP6" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP7" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R14" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND6" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R15" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND7" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R17" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C6" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND8" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP4" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R11" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="SUP8" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="IC1" library="SamacSys_Parts" deviceset="MCP1640" device=""/>
<part name="R18" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C5" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="GND9" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND10" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND11" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C3" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="GND12" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="R20" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="R19" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND13" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP9" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="SUP10" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="R16" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="Q2" library="SamacSys_Parts" deviceset="P-MOSFET" device=""/>
<part name="Q1" library="SamacSys_Parts" deviceset="N-MOSFET" device=""/>
<part name="L1" library="SamacSys_Parts" deviceset="INDUCTOR-0520-0630" device=""/>
<part name="RD1" library="SamacSys_Parts" deviceset="REED-DIP" device=""/>
<part name="D1" library="SamacSys_Parts" deviceset="DIODE-0603" device=""/>
<part name="D2" library="SamacSys_Parts" deviceset="DIODE-0603" device=""/>
<part name="GND14" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="C4" library="SamacSys_Parts" deviceset="CAPACITOR-0603" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="BAT1" library="SamacSys_Parts" deviceset="AAA-CONTACTS" device=""/>
<part name="BAT2" library="SamacSys_Parts" deviceset="AAA-CONTACTS" device=""/>
<part name="BAT3" library="SamacSys_Parts" deviceset="AAA-CONTACTS" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND15" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND16" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="CON1" library="SamacSys_Parts" deviceset="HEADER-6P-2R-DIP-2_54MM" device=""/>
<part name="R13" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="GND17" library="SamacSys_Parts" deviceset="GND" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="MOD1" gate="G$1" x="-15.24" y="7.62" smashed="yes">
<attribute name="NAME" x="-2.286" y="11.176" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-8.89" y="12.954" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SW1" gate="G$1" x="-27.94" y="-25.4" smashed="yes">
<attribute name="NAME" x="-24.13" y="-21.59" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-25.146" y="-20.066" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="C1" gate="G$1" x="-27.94" y="12.7" smashed="yes">
<attribute name="NAME" x="-23.114" y="9.652" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.13" y="17.526" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="LED1" gate="G$1" x="-58.42" y="-15.24" smashed="yes">
<attribute name="NAME" x="-51.308" y="-11.43" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-53.086" y="-9.906" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R1" gate="G$1" x="-27.94" y="7.62" smashed="yes">
<attribute name="NAME" x="-28.448" y="8.636" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="11.43" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R2" gate="G$1" x="-27.94" y="5.08" smashed="yes">
<attribute name="NAME" x="-28.448" y="6.096" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="8.89" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R3" gate="G$1" x="-27.94" y="2.54" smashed="yes">
<attribute name="NAME" x="-28.448" y="3.556" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="6.35" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R4" gate="G$1" x="-27.94" y="-2.54" smashed="yes">
<attribute name="NAME" x="-28.448" y="-1.524" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="1.27" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R5" gate="G$1" x="-27.94" y="-5.08" smashed="yes">
<attribute name="NAME" x="-28.448" y="-4.064" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="-1.27" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R6" gate="G$1" x="-27.94" y="-7.62" smashed="yes">
<attribute name="NAME" x="-28.448" y="-6.604" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-24.892" y="-3.81" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R7" gate="G$1" x="22.86" y="-7.62" smashed="yes">
<attribute name="NAME" x="33.528" y="-6.604" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="25.908" y="-3.81" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R8" gate="G$1" x="22.86" y="-5.08" smashed="yes">
<attribute name="NAME" x="33.274" y="-4.064" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="25.908" y="-1.27" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R9" gate="G$1" x="60.96" y="-5.08" smashed="yes" rot="R180">
<attribute name="NAME" x="61.214" y="-4.064" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="57.912" y="-8.89" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="R10" gate="G$1" x="22.86" y="0" smashed="yes">
<attribute name="NAME" x="33.274" y="1.016" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="25.908" y="3.81" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R12" gate="G$1" x="22.86" y="2.54" smashed="yes">
<attribute name="NAME" x="33.274" y="3.556" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="25.908" y="6.35" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND1" gate="1" x="-30.48" y="12.7" smashed="yes" rot="R270">
<attribute name="VALUE" x="-33.528" y="15.24" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND2" gate="1" x="17.78" y="-12.7" smashed="yes">
<attribute name="VALUE" x="15.24" y="-15.748" size="1.778" layer="96"/>
</instance>
<instance part="SUP1" gate="G$1" x="-30.48" y="7.62" smashed="yes" rot="R90">
<attribute name="VALUE" x="-32.385" y="5.715" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP2" gate="G$1" x="-30.48" y="2.54" smashed="yes" rot="R90">
<attribute name="VALUE" x="-32.385" y="0.635" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP3" gate="G$1" x="-17.78" y="-10.16" smashed="yes" rot="R90">
<attribute name="VALUE" x="-19.685" y="-12.065" size="1.778" layer="96" rot="R90"/>
</instance>
<instance part="SUP5" gate="G$1" x="63.5" y="-5.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="65.405" y="-3.175" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND3" gate="1" x="68.58" y="0" smashed="yes">
<attribute name="VALUE" x="66.04" y="-3.048" size="1.778" layer="96"/>
</instance>
<instance part="SUP6" gate="G$1" x="71.12" y="7.62" smashed="yes" rot="R270">
<attribute name="VALUE" x="73.025" y="9.525" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="SUP7" gate="G$1" x="-38.1" y="-12.7" smashed="yes">
<attribute name="VALUE" x="-40.005" y="-10.795" size="1.778" layer="96"/>
</instance>
<instance part="GND5" gate="1" x="76.2" y="-50.8" smashed="yes">
<attribute name="VALUE" x="73.66" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="R14" gate="G$1" x="-30.48" y="-35.56" smashed="yes">
<attribute name="NAME" x="-26.416" y="-37.846" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-27.432" y="-31.75" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="GND6" gate="1" x="-12.7" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-15.24" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="R15" gate="G$1" x="-7.62" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="-5.588" y="-41.402" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-11.43" y="-42.672" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND7" gate="1" x="-7.62" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-10.16" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="R17" gate="G$1" x="17.78" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="15.494" y="-43.434" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="13.97" y="-42.672" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C6" gate="G$1" x="22.86" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="21.844" y="-43.688" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="18.034" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND8" gate="1" x="35.56" y="-10.16" smashed="yes">
<attribute name="VALUE" x="33.02" y="-13.208" size="1.778" layer="96"/>
</instance>
<instance part="SUP4" gate="G$1" x="38.1" y="-5.08" smashed="yes" rot="R270">
<attribute name="VALUE" x="40.005" y="-3.175" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="R11" gate="G$1" x="22.86" y="-12.7" smashed="yes">
<attribute name="NAME" x="32.766" y="-11.684" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="25.908" y="-8.89" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP8" gate="G$1" x="38.1" y="-12.7" smashed="yes" rot="R270">
<attribute name="VALUE" x="40.005" y="-10.795" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="IC1" gate="G$1" x="35.56" y="-27.94" smashed="yes">
<attribute name="NAME" x="47.244" y="-24.13" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="40.894" y="-22.606" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R18" gate="G$1" x="30.48" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="28.194" y="-41.656" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="26.67" y="-42.672" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C5" gate="G$1" x="35.56" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="34.544" y="-43.688" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="30.734" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND9" gate="1" x="30.48" y="-50.8" smashed="yes">
<attribute name="VALUE" x="27.94" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND10" gate="1" x="35.56" y="-50.8" smashed="yes">
<attribute name="VALUE" x="33.02" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND11" gate="1" x="35.56" y="-30.48" smashed="yes" rot="R270">
<attribute name="VALUE" x="32.512" y="-27.94" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="C3" gate="G$1" x="71.12" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="70.104" y="-43.688" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="66.294" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND12" gate="1" x="71.12" y="-50.8" smashed="yes">
<attribute name="VALUE" x="68.58" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="R20" gate="G$1" x="66.04" y="-25.4" smashed="yes" rot="R90">
<attribute name="NAME" x="68.326" y="-21.336" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="62.23" y="-22.352" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="R19" gate="G$1" x="66.04" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="63.754" y="-41.91" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="62.23" y="-42.672" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND13" gate="1" x="66.04" y="-50.8" smashed="yes">
<attribute name="VALUE" x="63.5" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="SUP9" gate="G$1" x="66.04" y="-10.16" smashed="yes">
<attribute name="VALUE" x="64.135" y="-8.255" size="1.778" layer="96"/>
</instance>
<instance part="SUP10" gate="G$1" x="71.12" y="-22.86" smashed="yes">
<attribute name="VALUE" x="69.215" y="-20.955" size="1.778" layer="96"/>
</instance>
<instance part="R16" gate="G$1" x="-5.08" y="-33.02" smashed="yes">
<attribute name="NAME" x="-0.762" y="-30.734" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-2.032" y="-29.21" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="Q2" gate="G$1" x="7.62" y="-40.64" smashed="yes">
<attribute name="NAME" x="13.208" y="-35.306" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="13.208" y="-38.862" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="Q1" gate="G$1" x="-17.78" y="-40.64" smashed="yes">
<attribute name="NAME" x="-11.938" y="-35.052" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-12.192" y="-38.862" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="L1" gate="G$1" x="43.18" y="-20.32" smashed="yes">
<attribute name="NAME" x="48.26" y="-18.288" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="45.212" y="-16.51" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="RD1" gate="G$1" x="-7.62" y="-20.32" smashed="yes" rot="R270">
<attribute name="NAME" x="-5.334" y="-24.384" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-3.81" y="-22.86" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="D1" gate="G$1" x="30.48" y="-25.4" smashed="yes" rot="R180">
<attribute name="NAME" x="25.654" y="-23.114" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="28.956" y="-29.21" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="D2" gate="G$1" x="30.48" y="-30.48" smashed="yes" rot="R180">
<attribute name="NAME" x="25.654" y="-28.194" size="1.778" layer="95" rot="R180" align="center-left"/>
<attribute name="VALUE" x="28.956" y="-34.29" size="1.778" layer="96" rot="R180" align="center-left"/>
</instance>
<instance part="GND14" gate="1" x="81.28" y="-50.8" smashed="yes">
<attribute name="VALUE" x="78.74" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="C4" gate="G$1" x="76.2" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="75.184" y="-43.688" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="71.374" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C2" gate="G$1" x="81.28" y="-45.72" smashed="yes" rot="R90">
<attribute name="NAME" x="80.264" y="-43.688" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="76.454" y="-41.91" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="BAT1" gate="G$1" x="-35.56" y="-33.02" smashed="yes" rot="R270">
<attribute name="NAME" x="-34.29" y="-40.894" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-30.734" y="-36.83" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="BAT2" gate="G$1" x="-40.64" y="-33.02" smashed="yes" rot="R270">
<attribute name="NAME" x="-39.37" y="-40.894" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-35.814" y="-36.83" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="BAT3" gate="G$1" x="-45.72" y="-33.02" smashed="yes" rot="R270">
<attribute name="NAME" x="-44.45" y="-40.894" size="1.778" layer="95" rot="R270" align="center-left"/>
<attribute name="VALUE" x="-40.894" y="-36.83" size="1.778" layer="96" rot="R270" align="center-left"/>
</instance>
<instance part="GND4" gate="1" x="-35.56" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-38.1" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND15" gate="1" x="-40.64" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-43.18" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="GND16" gate="1" x="-45.72" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-48.26" y="-53.848" size="1.778" layer="96"/>
</instance>
<instance part="CON1" gate="G$1" x="50.8" y="7.62" smashed="yes">
<attribute name="NAME" x="55.88" y="11.176" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="55.88" y="12.954" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="R13" gate="G$1" x="-30.48" y="-48.26" smashed="yes" rot="R90">
<attribute name="NAME" x="-28.194" y="-44.196" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-34.29" y="-45.212" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="GND17" gate="1" x="-30.48" y="-50.8" smashed="yes">
<attribute name="VALUE" x="-33.02" y="-53.848" size="1.778" layer="96"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="N$2" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="ADC"/>
<pinref part="R2" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="EN"/>
<pinref part="R3" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO14"/>
<pinref part="R4" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO12"/>
<pinref part="R5" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO13"/>
<pinref part="R6" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO15"/>
<pinref part="R7" gate="G$1" pin="1"/>
<wire x1="22.86" y1="-7.62" x2="17.78" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO2"/>
<pinref part="R8" gate="G$1" pin="1"/>
<wire x1="22.86" y1="-5.08" x2="17.78" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO4"/>
<pinref part="R10" gate="G$1" pin="1"/>
<wire x1="22.86" y1="0" x2="20.32" y2="0" width="0.1524" layer="91"/>
<pinref part="R11" gate="G$1" pin="1"/>
<wire x1="20.32" y1="0" x2="17.78" y2="0" width="0.1524" layer="91"/>
<wire x1="22.86" y1="-12.7" x2="20.32" y2="-12.7" width="0.1524" layer="91"/>
<wire x1="20.32" y1="-12.7" x2="20.32" y2="0" width="0.1524" layer="91"/>
<junction x="20.32" y="0"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO5"/>
<pinref part="R12" gate="G$1" pin="1"/>
<wire x1="22.86" y1="2.54" x2="17.78" y2="2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="C1" gate="G$1" pin="1"/>
<pinref part="GND1" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="GND"/>
<pinref part="GND2" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R15" gate="G$1" pin="1"/>
<pinref part="GND7" gate="1" pin="GND"/>
<wire x1="-7.62" y1="-48.26" x2="-7.62" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R7" gate="G$1" pin="2"/>
<pinref part="GND8" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R18" gate="G$1" pin="1"/>
<pinref part="GND9" gate="1" pin="GND"/>
<wire x1="30.48" y1="-48.26" x2="30.48" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C5" gate="G$1" pin="1"/>
<pinref part="GND10" gate="1" pin="GND"/>
<wire x1="35.56" y1="-48.26" x2="35.56" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="GND"/>
<pinref part="GND11" gate="1" pin="GND"/>
<wire x1="35.56" y1="-30.48" x2="38.1" y2="-30.48" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C3" gate="G$1" pin="1"/>
<pinref part="GND12" gate="1" pin="GND"/>
<wire x1="71.12" y1="-48.26" x2="71.12" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="R19" gate="G$1" pin="1"/>
<pinref part="GND13" gate="1" pin="GND"/>
<wire x1="66.04" y1="-48.26" x2="66.04" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="Q1" gate="G$1" pin="S"/>
<pinref part="GND6" gate="1" pin="GND"/>
<wire x1="-12.7" y1="-43.18" x2="-12.7" y2="-48.26" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND5" gate="1" pin="GND"/>
<pinref part="C4" gate="G$1" pin="1"/>
<wire x1="76.2" y1="-48.26" x2="76.2" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="GND14" gate="1" pin="GND"/>
<pinref part="C2" gate="G$1" pin="1"/>
<wire x1="81.28" y1="-48.26" x2="81.28" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BAT1" gate="G$1" pin="-"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="-35.56" y1="-48.26" x2="-35.56" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BAT2" gate="G$1" pin="-"/>
<pinref part="GND15" gate="1" pin="GND"/>
<wire x1="-40.64" y1="-48.26" x2="-40.64" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BAT3" gate="G$1" pin="-"/>
<pinref part="GND16" gate="1" pin="GND"/>
<wire x1="-45.72" y1="-48.26" x2="-45.72" y2="-45.72" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="6"/>
<pinref part="GND3" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R13" gate="G$1" pin="1"/>
<pinref part="GND17" gate="1" pin="GND"/>
</segment>
</net>
<net name="VCC" class="0">
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R3" gate="G$1" pin="1"/>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="MOD1" gate="G$1" pin="VCC"/>
<pinref part="SUP3" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R9" gate="G$1" pin="1"/>
<pinref part="SUP5" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="LED1" gate="G$1" pin="A"/>
<pinref part="SUP7" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R8" gate="G$1" pin="2"/>
<pinref part="SUP4" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R11" gate="G$1" pin="2"/>
<pinref part="SUP8" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="R20" gate="G$1" pin="2"/>
<pinref part="SUP9" gate="G$1" pin="VCC"/>
</segment>
<segment>
<pinref part="IC1" gate="G$1" pin="VOUT"/>
<wire x1="63.5" y1="-30.48" x2="71.12" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="C3" gate="G$1" pin="2"/>
<wire x1="71.12" y1="-30.48" x2="71.12" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="SUP10" gate="G$1" pin="VCC"/>
<wire x1="71.12" y1="-25.4" x2="71.12" y2="-30.48" width="0.1524" layer="91"/>
<junction x="71.12" y="-30.48"/>
<wire x1="71.12" y1="-30.48" x2="76.2" y2="-30.48" width="0.1524" layer="91"/>
<wire x1="76.2" y1="-30.48" x2="76.2" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="C4" gate="G$1" pin="2"/>
</segment>
<segment>
<pinref part="CON1" gate="G$1" pin="4"/>
<pinref part="SUP6" gate="G$1" pin="VCC"/>
</segment>
</net>
<net name="N$12" class="0">
<segment>
<pinref part="R10" gate="G$1" pin="2"/>
<wire x1="35.56" y1="0" x2="40.64" y2="0" width="0.1524" layer="91"/>
<wire x1="40.64" y1="0" x2="40.64" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="40.64" y1="-15.24" x2="-12.7" y2="-15.24" width="0.1524" layer="91"/>
<wire x1="-12.7" y1="-15.24" x2="-12.7" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="Q1" gate="G$1" pin="D"/>
</segment>
</net>
<net name="N$16" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="TXD0"/>
<wire x1="17.78" y1="7.62" x2="43.18" y2="7.62" width="0.1524" layer="91"/>
<wire x1="43.18" y1="7.62" x2="43.18" y2="5.08" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="2"/>
<wire x1="43.18" y1="5.08" x2="50.8" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$18" class="0">
<segment>
<wire x1="-35.56" y1="-2.54" x2="-35.56" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="B"/>
<wire x1="-35.56" y1="-17.78" x2="-38.1" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="R4" gate="G$1" pin="1"/>
<wire x1="-35.56" y1="-2.54" x2="-27.94" y2="-2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$19" class="0">
<segment>
<pinref part="R5" gate="G$1" pin="1"/>
<wire x1="-27.94" y1="-5.08" x2="-58.42" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="R"/>
<wire x1="-58.42" y1="-5.08" x2="-58.42" y2="-15.24" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$20" class="0">
<segment>
<pinref part="R6" gate="G$1" pin="1"/>
<wire x1="-27.94" y1="-7.62" x2="-60.96" y2="-7.62" width="0.1524" layer="91"/>
<pinref part="LED1" gate="G$1" pin="G"/>
<wire x1="-60.96" y1="-7.62" x2="-60.96" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="-60.96" y1="-17.78" x2="-58.42" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$21" class="0">
<segment>
<pinref part="SW1" gate="G$1" pin="2"/>
<wire x1="-27.94" y1="-27.94" x2="-30.48" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="-30.48" y1="-27.94" x2="-30.48" y2="-17.78" width="0.1524" layer="91"/>
<wire x1="-30.48" y1="-17.78" x2="17.78" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="R14" gate="G$1" pin="1"/>
<wire x1="-30.48" y1="-27.94" x2="-30.48" y2="-33.02" width="0.1524" layer="91"/>
<junction x="-30.48" y="-27.94"/>
<pinref part="D1" gate="G$1" pin="A"/>
<wire x1="-30.48" y1="-33.02" x2="-30.48" y2="-35.56" width="0.1524" layer="91"/>
<wire x1="17.78" y1="-17.78" x2="17.78" y2="-25.4" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="3"/>
<wire x1="-15.24" y1="-27.94" x2="-15.24" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="-33.02" x2="-30.48" y2="-33.02" width="0.1524" layer="91"/>
<junction x="-30.48" y="-33.02"/>
<pinref part="R13" gate="G$1" pin="2"/>
<junction x="-30.48" y="-35.56"/>
</segment>
</net>
<net name="N$25" class="0">
<segment>
<pinref part="R17" gate="G$1" pin="1"/>
<pinref part="C6" gate="G$1" pin="1"/>
<wire x1="17.78" y1="-45.72" x2="22.86" y2="-45.72" width="0.1524" layer="91"/>
<pinref part="Q2" gate="G$1" pin="D"/>
<wire x1="12.7" y1="-43.18" x2="12.7" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="12.7" y1="-45.72" x2="17.78" y2="-45.72" width="0.1524" layer="91"/>
<junction x="17.78" y="-45.72"/>
</segment>
</net>
<net name="N$14" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="GPIO0"/>
<wire x1="17.78" y1="-2.54" x2="48.26" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="R9" gate="G$1" pin="2"/>
<wire x1="48.26" y1="-5.08" x2="48.26" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="1"/>
<wire x1="48.26" y1="7.62" x2="50.8" y2="7.62" width="0.1524" layer="91"/>
<wire x1="48.26" y1="-2.54" x2="48.26" y2="7.62" width="0.1524" layer="91"/>
<junction x="48.26" y="-2.54"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="SW"/>
<wire x1="35.56" y1="-27.94" x2="35.56" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="35.56" y1="-20.32" x2="43.18" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="L1" gate="G$1" pin="1"/>
</segment>
</net>
<net name="N$27" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="VFB"/>
<pinref part="R19" gate="G$1" pin="2"/>
<wire x1="63.5" y1="-33.02" x2="66.04" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="R20" gate="G$1" pin="1"/>
<wire x1="66.04" y1="-25.4" x2="66.04" y2="-33.02" width="0.1524" layer="91"/>
<junction x="66.04" y="-33.02"/>
</segment>
</net>
<net name="N$26" class="0">
<segment>
<pinref part="C6" gate="G$1" pin="2"/>
<pinref part="R17" gate="G$1" pin="2"/>
<wire x1="17.78" y1="-33.02" x2="22.86" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="D2" gate="G$1" pin="A"/>
<wire x1="17.78" y1="-30.48" x2="17.78" y2="-33.02" width="0.1524" layer="91"/>
<junction x="17.78" y="-33.02"/>
</segment>
</net>
<net name="N$24" class="0">
<segment>
<pinref part="R16" gate="G$1" pin="2"/>
<pinref part="Q2" gate="G$1" pin="G"/>
<wire x1="7.62" y1="-33.02" x2="7.62" y2="-35.56" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$28" class="0">
<segment>
<pinref part="R14" gate="G$1" pin="2"/>
<pinref part="Q1" gate="G$1" pin="G"/>
</segment>
</net>
<net name="N$23" class="0">
<segment>
<pinref part="R15" gate="G$1" pin="2"/>
<pinref part="R16" gate="G$1" pin="1"/>
<pinref part="RD1" gate="G$1" pin="2"/>
<junction x="-7.62" y="-33.02"/>
<wire x1="-5.08" y1="-33.02" x2="-7.62" y2="-33.02" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$29" class="0">
<segment>
<wire x1="-33.02" y1="5.08" x2="-33.02" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="R2" gate="G$1" pin="1"/>
<wire x1="-27.94" y1="5.08" x2="-33.02" y2="5.08" width="0.1524" layer="91"/>
<pinref part="SW1" gate="G$1" pin="1"/>
<wire x1="-33.02" y1="-20.32" x2="-27.94" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="-27.94" y1="-25.4" x2="-27.94" y2="-20.32" width="0.1524" layer="91"/>
<pinref part="Q2" gate="G$1" pin="S"/>
<wire x1="12.7" y1="-20.32" x2="12.7" y2="-22.86" width="0.1524" layer="91"/>
<wire x1="12.7" y1="-22.86" x2="12.7" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-27.94" y1="-20.32" x2="-15.24" y2="-20.32" width="0.1524" layer="91"/>
<junction x="-27.94" y="-20.32"/>
<pinref part="RD1" gate="G$1" pin="1"/>
<wire x1="-15.24" y1="-20.32" x2="-7.62" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="-7.62" y1="-20.32" x2="12.7" y2="-20.32" width="0.1524" layer="91"/>
<junction x="-7.62" y="-20.32"/>
<pinref part="IC1" gate="G$1" pin="VIN"/>
<wire x1="55.88" y1="-20.32" x2="63.5" y2="-20.32" width="0.1524" layer="91"/>
<wire x1="63.5" y1="-20.32" x2="63.5" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="L1" gate="G$1" pin="2"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="63.5" y1="-22.86" x2="63.5" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="63.5" y1="-27.94" x2="81.28" y2="-27.94" width="0.1524" layer="91"/>
<wire x1="81.28" y1="-27.94" x2="81.28" y2="-33.02" width="0.1524" layer="91"/>
<junction x="63.5" y="-27.94"/>
<wire x1="12.7" y1="-22.86" x2="63.5" y2="-22.86" width="0.1524" layer="91"/>
<junction x="12.7" y="-22.86"/>
<junction x="63.5" y="-22.86"/>
<pinref part="SW1" gate="G$1" pin="4"/>
<wire x1="-15.24" y1="-25.4" x2="-15.24" y2="-20.32" width="0.1524" layer="91"/>
<junction x="-15.24" y="-20.32"/>
<wire x1="-33.02" y1="-20.32" x2="-33.02" y2="-33.02" width="0.1524" layer="91"/>
<junction x="-33.02" y="-20.32"/>
<pinref part="BAT1" gate="G$1" pin="+"/>
<pinref part="BAT3" gate="G$1" pin="+"/>
<wire x1="-45.72" y1="-33.02" x2="-40.64" y2="-33.02" width="0.1524" layer="91"/>
<pinref part="BAT2" gate="G$1" pin="+"/>
<junction x="-40.64" y="-33.02"/>
<wire x1="-40.64" y1="-33.02" x2="-35.56" y2="-33.02" width="0.1524" layer="91"/>
<wire x1="-33.02" y1="-33.02" x2="-35.56" y2="-33.02" width="0.1524" layer="91"/>
<junction x="-35.56" y="-33.02"/>
</segment>
</net>
<net name="N$11" class="0">
<segment>
<pinref part="R12" gate="G$1" pin="2"/>
<wire x1="35.56" y1="2.54" x2="43.18" y2="2.54" width="0.1524" layer="91"/>
<wire x1="43.18" y1="2.54" x2="43.18" y2="-17.78" width="0.1524" layer="91"/>
<pinref part="C5" gate="G$1" pin="2"/>
<pinref part="IC1" gate="G$1" pin="EN"/>
<junction x="35.56" y="-33.02"/>
<pinref part="R18" gate="G$1" pin="2"/>
<wire x1="30.48" y1="-33.02" x2="35.56" y2="-33.02" width="0.1524" layer="91"/>
<junction x="30.48" y="-33.02"/>
<wire x1="30.48" y1="-17.78" x2="30.48" y2="-25.4" width="0.1524" layer="91"/>
<pinref part="D1" gate="G$1" pin="K"/>
<junction x="30.48" y="-25.4"/>
<wire x1="30.48" y1="-25.4" x2="30.48" y2="-30.48" width="0.1524" layer="91"/>
<pinref part="D2" gate="G$1" pin="K"/>
<wire x1="30.48" y1="-30.48" x2="30.48" y2="-33.02" width="0.1524" layer="91"/>
<junction x="30.48" y="-30.48"/>
<wire x1="43.18" y1="-17.78" x2="30.48" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$15" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RST"/>
<pinref part="R1" gate="G$1" pin="2"/>
<junction x="-15.24" y="7.62"/>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="-15.24" y1="12.7" x2="-15.24" y2="7.62" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="12.7" x2="-15.24" y2="15.24" width="0.1524" layer="91"/>
<junction x="-15.24" y="12.7"/>
<wire x1="-15.24" y1="15.24" x2="45.72" y2="15.24" width="0.1524" layer="91"/>
<wire x1="45.72" y1="15.24" x2="45.72" y2="2.54" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="3"/>
<wire x1="45.72" y1="2.54" x2="50.8" y2="2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$13" class="0">
<segment>
<pinref part="MOD1" gate="G$1" pin="RXD0"/>
<wire x1="17.78" y1="5.08" x2="20.32" y2="5.08" width="0.1524" layer="91"/>
<wire x1="20.32" y1="5.08" x2="20.32" y2="12.7" width="0.1524" layer="91"/>
<wire x1="20.32" y1="12.7" x2="73.66" y2="12.7" width="0.1524" layer="91"/>
<wire x1="73.66" y1="12.7" x2="73.66" y2="5.08" width="0.1524" layer="91"/>
<pinref part="CON1" gate="G$1" pin="5"/>
<wire x1="73.66" y1="5.08" x2="68.58" y2="5.08" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
