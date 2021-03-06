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
<package name="REED-7MM-SMD">
<description>&lt;b&gt;825433-5&lt;/b&gt;&lt;br&gt;
</description>
<wire x1="-3.5" y1="0.9" x2="-3.5" y2="-0.9" width="0.1" layer="51"/>
<wire x1="-3.5" y1="-0.9" x2="3.5" y2="-0.9" width="0.1" layer="51"/>
<wire x1="3.5" y1="-0.9" x2="3.5" y2="0.9" width="0.1" layer="51"/>
<wire x1="3.5" y1="0.9" x2="-3.5" y2="0.9" width="0.1" layer="51"/>
<wire x1="-3.5" y1="0.9" x2="3.5" y2="0.9" width="0.2" layer="21"/>
<wire x1="3.5" y1="-0.9" x2="-3.5" y2="-0.9" width="0.2" layer="21"/>
<text x="-3" y="1.15" size="1.27" layer="25">&gt;NAME</text>
<text x="-3" y="2.45" size="1.27" layer="27">&gt;VALUE</text>
<smd name="P$1" x="-3.5" y="0" dx="2" dy="1" layer="1"/>
<smd name="P$2" x="3.5" y="0.05" dx="2" dy="1" layer="1"/>
</package>
</packages>
<symbols>
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
<symbol name="PIC10F206">
<wire x1="5.08" y1="2.54" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="22.86" y2="2.54" width="0.254" layer="94"/>
<wire x1="22.86" y1="-7.62" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<wire x1="5.08" y1="2.54" x2="5.08" y2="-7.62" width="0.254" layer="94"/>
<text x="5.334" y="3.556" size="1.778" layer="95" align="center-left">&gt;NAME</text>
<text x="5.334" y="5.334" size="1.778" layer="96" align="center-left">&gt;VALUE</text>
<pin name="GP0" x="0" y="0" length="middle" direction="pwr"/>
<pin name="VSS" x="0" y="-2.54" length="middle" direction="pwr"/>
<pin name="GP1" x="0" y="-5.08" length="middle"/>
<pin name="GP2" x="27.94" y="-5.08" length="middle" rot="R180"/>
<pin name="VDD" x="27.94" y="-2.54" length="middle" direction="pwr" rot="R180"/>
<pin name="GP3" x="27.94" y="0" length="middle" direction="pwr" rot="R180"/>
</symbol>
<symbol name="VCC">
<circle x="0" y="0.762" radius="0.762" width="0.254" layer="94"/>
<text x="-1.905" y="1.905" size="1.778" layer="96">&gt;VALUE</text>
<pin name="VCC" x="0" y="-2.54" visible="off" length="short" direction="sup" rot="R90"/>
</symbol>
<symbol name="GND">
<wire x1="-1.905" y1="0" x2="1.905" y2="0" width="0.254" layer="94"/>
<text x="-2.54" y="-3.048" size="1.778" layer="96">&gt;VALUE</text>
<pin name="GND" x="0" y="2.54" visible="off" length="short" direction="sup" rot="R270"/>
<wire x1="-1.27" y1="-0.508" x2="1.27" y2="-0.508" width="0.254" layer="94"/>
<wire x1="-0.508" y1="-1.016" x2="0.508" y2="-1.016" width="0.254" layer="94"/>
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
</symbols>
<devicesets>
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
<deviceset name="PIC10F206" prefix="IC" uservalue="yes">
<gates>
<gate name="G$1" symbol="PIC10F206" x="0" y="0"/>
</gates>
<devices>
<device name="" package="SOT23-6">
<connects>
<connect gate="G$1" pin="GP0" pad="1"/>
<connect gate="G$1" pin="GP1" pad="3"/>
<connect gate="G$1" pin="GP2" pad="4"/>
<connect gate="G$1" pin="GP3" pad="6"/>
<connect gate="G$1" pin="VDD" pad="5"/>
<connect gate="G$1" pin="VSS" pad="2"/>
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
<deviceset name="REED-7MM-SMD" prefix="SW" uservalue="yes">
<gates>
<gate name="G$1" symbol="REED" x="0" y="0"/>
</gates>
<devices>
<device name="" package="REED-7MM-SMD">
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
<part name="R1" library="SamacSys_Parts" deviceset="RESISTOR-0603" device=""/>
<part name="C1" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="C2" library="SamacSys_Parts" deviceset="CAPACITOR-0805" device=""/>
<part name="Q1" library="SamacSys_Parts" deviceset="N-MOSFET" device=""/>
<part name="IC1" library="SamacSys_Parts" deviceset="PIC10F206" device=""/>
<part name="SUP1" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND1" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND2" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND3" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SUP2" library="SamacSys_Parts" deviceset="VCC" device=""/>
<part name="GND4" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="GND5" library="SamacSys_Parts" deviceset="GND" device=""/>
<part name="SW1" library="SamacSys_Parts" deviceset="REED-7MM-SMD" device=""/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="R1" gate="G$1" x="-20.32" y="-15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-22.352" y="-12.192" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-24.13" y="-12.192" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C1" gate="G$1" x="17.78" y="-15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="14.732" y="-11.43" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="12.954" y="-11.43" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="C2" gate="G$1" x="22.86" y="-15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="19.812" y="-11.43" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="18.034" y="-11.43" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
<instance part="Q1" gate="G$1" x="-12.7" y="-15.24" smashed="yes">
<attribute name="NAME" x="-7.112" y="-11.684" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-7.112" y="-13.462" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="IC1" gate="G$1" x="-12.7" y="2.54" smashed="yes">
<attribute name="NAME" x="-7.366" y="6.096" size="1.778" layer="95" align="center-left"/>
<attribute name="VALUE" x="-7.366" y="7.874" size="1.778" layer="96" align="center-left"/>
</instance>
<instance part="SUP1" gate="G$1" x="25.4" y="2.54" smashed="yes" rot="R270">
<attribute name="VALUE" x="27.305" y="4.445" size="1.778" layer="96" rot="R270"/>
</instance>
<instance part="GND1" gate="1" x="-15.24" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-17.78" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND2" gate="1" x="-7.62" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-10.16" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND3" gate="1" x="-20.32" y="-20.32" smashed="yes">
<attribute name="VALUE" x="-22.86" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="SUP2" gate="G$1" x="-25.4" y="-17.78" smashed="yes" rot="R180">
<attribute name="VALUE" x="-23.495" y="-19.685" size="1.778" layer="96" rot="R180"/>
</instance>
<instance part="GND4" gate="1" x="17.78" y="-20.32" smashed="yes">
<attribute name="VALUE" x="15.24" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="GND5" gate="1" x="22.86" y="-20.32" smashed="yes">
<attribute name="VALUE" x="20.32" y="-23.368" size="1.778" layer="96"/>
</instance>
<instance part="SW1" gate="G$1" x="-25.4" y="-15.24" smashed="yes" rot="R90">
<attribute name="NAME" x="-27.432" y="-12.7" size="1.778" layer="95" rot="R90" align="center-left"/>
<attribute name="VALUE" x="-29.21" y="-12.7" size="1.778" layer="96" rot="R90" align="center-left"/>
</instance>
</instances>
<busses>
</busses>
<nets>
<net name="VCC" class="0">
<segment>
<pinref part="SUP1" gate="G$1" pin="VCC"/>
<pinref part="C2" gate="G$1" pin="2"/>
<wire x1="22.86" y1="-2.54" x2="22.86" y2="2.54" width="0.1524" layer="91"/>
<pinref part="IC1" gate="G$1" pin="GP3"/>
<wire x1="15.24" y1="2.54" x2="17.78" y2="2.54" width="0.1524" layer="91"/>
<junction x="22.86" y="2.54"/>
<pinref part="IC1" gate="G$1" pin="VDD"/>
<wire x1="17.78" y1="2.54" x2="22.86" y2="2.54" width="0.1524" layer="91"/>
<wire x1="15.24" y1="0" x2="17.78" y2="0" width="0.1524" layer="91"/>
<pinref part="C1" gate="G$1" pin="2"/>
<wire x1="17.78" y1="-2.54" x2="17.78" y2="0" width="0.1524" layer="91"/>
<wire x1="17.78" y1="0" x2="17.78" y2="2.54" width="0.1524" layer="91"/>
<junction x="17.78" y="0"/>
<junction x="17.78" y="2.54"/>
</segment>
<segment>
<pinref part="SUP2" gate="G$1" pin="VCC"/>
<pinref part="SW1" gate="G$1" pin="1"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="VSS"/>
<pinref part="GND1" gate="1" pin="GND"/>
<wire x1="-12.7" y1="0" x2="-15.24" y2="0" width="0.1524" layer="91"/>
<wire x1="-15.24" y1="0" x2="-15.24" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="Q1" gate="G$1" pin="S"/>
<pinref part="GND2" gate="1" pin="GND"/>
</segment>
<segment>
<pinref part="R1" gate="G$1" pin="1"/>
<pinref part="GND3" gate="1" pin="GND"/>
<wire x1="-20.32" y1="-17.78" x2="-20.32" y2="-15.24" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C1" gate="G$1" pin="1"/>
<pinref part="GND4" gate="1" pin="GND"/>
<wire x1="17.78" y1="-15.24" x2="17.78" y2="-17.78" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="C2" gate="G$1" pin="1"/>
<pinref part="GND5" gate="1" pin="GND"/>
<wire x1="22.86" y1="-15.24" x2="22.86" y2="-17.78" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="GP1"/>
<pinref part="Q1" gate="G$1" pin="G"/>
<wire x1="-12.7" y1="-2.54" x2="-12.7" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="IC1" gate="G$1" pin="GP0"/>
<wire x1="-12.7" y1="2.54" x2="-20.32" y2="2.54" width="0.1524" layer="91"/>
<wire x1="-20.32" y1="2.54" x2="-25.4" y2="2.54" width="0.1524" layer="91"/>
<wire x1="-25.4" y1="2.54" x2="-25.4" y2="-2.54" width="0.1524" layer="91"/>
<pinref part="R1" gate="G$1" pin="2"/>
<wire x1="-20.32" y1="-2.54" x2="-20.32" y2="2.54" width="0.1524" layer="91"/>
<junction x="-20.32" y="2.54"/>
<pinref part="SW1" gate="G$1" pin="2"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="Q1" gate="G$1" pin="D"/>
<wire x1="-7.62" y1="-7.62" x2="-2.54" y2="-7.62" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
</schematic>
</drawing>
</eagle>
