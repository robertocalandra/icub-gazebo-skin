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
public class GenerateXMLLeftArmRange {
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
		
		String pose_string="";
		
		// ArrayList of all collision names
		ArrayList<String> collision_names = new ArrayList<String>();
		String box_collision_name="";
		
		// Go through all skin Data and create appopriate gazebo model
		// make left-arm model
		BufferedReader br = new BufferedReader(new FileReader("positions/left_forearm_mesh.txt"));
        
        int CountDoubles=0;
        
        while((pose_content = br.readLine())!=null) {
        	
          	box_count++;
          	
        	if(!isInRange(box_count)) {
        		
        		pose_data=getDoublesFromString(pose_content);
        		
        		pose_data[1]+=0.14;
        		pose_data[2]+=0.02;
        		
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
		yarpConfigurationFile.addContent("model://icub/conf/gazebo_icub_left_forearm.ini");
		plugin = new Element("plugin");
		plugin.setAttribute("filename","libgazebo_yarp_skin.so");
		plugin.setAttribute("name","iCub_yarp_gazebo_plugin_Skin.so");
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
		xout.output(icub_skin, new FileWriter("skin_left_forearm.sdf"));
	}
	public static boolean isInRange(int counter) {
		// Left-Forearm
		// Define Ranges where no sensors at
		int[][] ranges = new int[10][2];
		
		/*
		 * Empty:
		 * - 193-204
		 * - 217-252
		 * - 265-288
		 * - 325-336
		 * - 361-384
		 */
		ranges[0][0]=193+1;
		ranges[0][1]=204;
				
		ranges[1][0]=217;
		ranges[1][1]=252;
		
		ranges[2][0]=265;
		ranges[2][1]=288;
		
		ranges[3][0]=325;
		ranges[3][1]=336;
		
		ranges[4][0]=361;
		ranges[4][1]=384;
		
	
		
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
