import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

import org.jdom2.*;
import org.jdom2.input.*;
import org.jdom2.output.*;
/**
 * 
 * @author Moritz Nakatenus, TU Darmstadt
 *
 */
public class GenerateXMLRightUpperArmRange {
	public static void main(String[] args) throws Exception {

	
		String pose_content = new String();
		String size_box="0.005 0.005 0.005";
		int box_count=0; //counter for every collision box
		
		Element size = new Element("size");
		Element box = new Element("box");
		Element geometry = new Element("geometry");
		Element collision = new Element("collision");
		Element sensor = new Element("sensor");
		Element visual = new Element("visual");
		Element collision1 = new Element("collision");
		Element pose = new Element("pose");
		Element sdf = new Element("sdf");
	
		Element inertial_pose;
		
		// Elements for sensor
		Element always_on = new Element("alyways_on");
		Element update_rate = new Element("update_rate");
		Element visualize = new Element("visualize");
		Element plugin = new Element("plugin");
		Element yarpConfigurationFile = new Element("yarpConfigurationFile");
		Element contact = new Element("contact");
		
		sdf.setAttribute("version", "1.4");
		
		double[] pose_data = new double[6];
		double hyp=0; //hypothenuse
		
		String pose_string="";
		
		// ArrayList of all collision names
		ArrayList<String> collision_names = new ArrayList<String>();
		String box_collision_name="";
		
		// Go through all skin Data and create appopriate gazebo model
		// make left-arm model
		BufferedReader br = new BufferedReader(new FileReader("positions/right_arm_mesh.txt"));
        
        int CountDoubles=0;
        
        while((pose_content = br.readLine())!=null) {
        	
          	box_count++;
          	
        	if(!isInRange(box_count)) {
        		
        		pose_data=getDoublesFromString(pose_content);
        		
        		pose_data[0]+=0.01;
        		pose_data[1]-=0.01;
        		
        		for(int i=0; i<6; i++)
        			pose_string+=pose_data[i]+" ";
              	
    			pose = new Element("pose");
    			pose.addContent(pose_string);
    			
    			size = new Element("size");
    			size.addContent(size_box);
    			
    			box = new Element("box");
    			box.addContent(size);
    			
    			geometry = new Element("geometry");
    			geometry.addContent(box);
    			
    			collision = new Element("collision");
    			box_collision_name="box_collision"+box_count;
    			collision.addContent(box_collision_name);
    			collision_names.add(box_collision_name);
    			
    			visual = new Element("visual");
    			visual.setAttribute("name","visual"+box_count);
    			visual.addContent(pose);
    			visual.addContent(geometry);
    			
    			collision1 = new Element("collision");
    			collision1.setAttribute("name",box_collision_name);
    			geometry = new Element("geometry"); // Overwrite
    			box = new Element("box");
    			size = new Element("size");
    			size.addContent(size_box);
    			box.addContent(size);
    			geometry.addContent(box);
    			pose = new Element("pose");
    			pose.addContent(pose_string);
    			collision1.addContent(pose);
    			collision1.addContent(geometry);
    			
    			
    			sdf.addContent(collision1);
    			sdf.addContent(visual);
    			
    			pose_string="";
    			
    			if(CountDoubles!=0)
    				System.out.println(CountDoubles);
    			CountDoubles=0;
        	} else {
        		CountDoubles++;
        	}

        }

		contact = new Element("contact");
		
		// Set contact collisions
		for(int i=0; i < collision_names.size(); i++) {
			collision=new Element("collision");
			collision.addContent(collision_names.get(i));
			contact.addContent(collision);
		}
		
        always_on = new Element("always_on");
		always_on.addContent("1");
		update_rate = new Element("update_rate");
		update_rate.addContent("100");
		visualize = new Element("visualize");
		visualize.addContent("1");
		yarpConfigurationFile= new Element("yarpConfigurationFile");
		yarpConfigurationFile.addContent("model://icub/conf/gazebo_icub_left_forearm_skin.ini");
		plugin = new Element("plugin");
		plugin.setAttribute("filename","libgazebo_yarp_contact.so");
		plugin.setAttribute("name","iCub_yarp_gazebo_plugin_Contact.so");
		plugin.addContent(yarpConfigurationFile);
		sensor = new Element("sensor");
		sensor.setAttribute("name","my_contact"+box_count);
		sensor.setAttribute("type","contact");
		sensor.addContent(always_on);
		sensor.addContent(update_rate);
		sensor.addContent(visualize);
		sensor.addContent(plugin);
		sensor.addContent(contact);
		
		sdf.addContent(sensor);
		
		Document icub_skin = new Document(sdf);
		
		XMLOutputter xout = new XMLOutputter(Format.getPrettyFormat());
		xout.output(icub_skin, new FileWriter("skin_right_arm.sdf"));
	}
	public static boolean isInRange(int counter) {
		// Left-Forearm
		// Define Ranges where no sensors at
		int[][] ranges = new int[22][2];
		

		ranges[0][0]=1;
		ranges[0][1]=36;
				
		ranges[1][0]=97;
		ranges[1][1]=108;
		
		ranges[2][0]=145;
		ranges[2][1]=192;
		
		ranges[3][0]=217;
		ranges[3][1]=264;
		
		ranges[4][0]=277;
		ranges[4][1]=288;
		
		ranges[5][0]=493;
		ranges[5][1]=528;
		
		ranges[6][0]=553;
		ranges[6][1]=576;
		
		ranges[7][0]=589;
		ranges[7][1]=636;
		
		ranges[8][0]=649;
		ranges[8][1]=672;

		ranges[9][0]=721;
		ranges[9][1]=732;
		
		ranges[10][0]=745;
		ranges[10][1]=756;
		
		for(int i=0; i < ranges.length/2; i++) {
			
			
			if((counter>=ranges[i][0])&&(counter<=ranges[i][1]))
				return true;
		}
		
		return false;
	}
	
	public static double[] getDoublesFromString(String string) {
		double[] doubles = new double[6];
		
		String tmpString="";
		int count=0; //float counter
		
		for(int i=0; i < string.length(); i++) {
			
			if(!(string.charAt(i)==' ')) {
				
				tmpString+=string.charAt(i);
			} else {
				if(!tmpString.equals("")) {
					
					doubles[count]=Double.parseDouble(tmpString);
					count++;
				}
				
				tmpString="";
			}
		}
		
		doubles[5]=Double.parseDouble(tmpString);
		
		return doubles;
	}
}
