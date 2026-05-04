import xml.etree.ElementTree as ET
import csv
import sys
import os

def convert(xml_file, csv_file):
    print(f"Converting {xml_file} to {csv_file}...")
    try:
        context = ET.iterparse(xml_file, events=('end',))
        
        with open(csv_file, 'w', newline='', encoding='utf-8') as cout:
            writer = csv.writer(cout, delimiter=';')
            writer.writerow(['timestep_time', 'vehicle_id', 'vehicle_x', 'vehicle_y', 'vehicle_lane', 'vehicle_speed'])
            
            current_time = "0.00"
            count = 0
            
            for event, elem in context:
                if elem.tag == 'timestep':
                    if 'time' in elem.attrib:
                        current_time = elem.attrib['time']
                    elem.clear()
                    
                elif elem.tag == 'vehicle':
                    writer.writerow([
                        current_time,
                        elem.attrib.get('id', ''),
                        elem.attrib.get('x', ''),
                        elem.attrib.get('y', ''),
                        elem.attrib.get('lane', ''),
                        elem.attrib.get('speed', '')
                    ])
                    count += 1
                    elem.clear()  # prevent memory leak
                    
                # Clean up parent references to save memory periodically
                if count % 100000 == 0:
                     print(f"Processed {count} vehicle records...")
                     
        print(f"Conversion complete! Processed {count} vehicle positions.")
    except Exception as e:
        print(f"Error during conversion: {e}")

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(base_dir, 'data', 'simulation', 'trajectories.xml')
    csv_path = os.path.join(base_dir, 'data', 'simulation', 'trajectories.csv')
    convert(xml_path, csv_path)
