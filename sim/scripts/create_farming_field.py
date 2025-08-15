#!/usr/bin/env python3
"""
Create Realistic Farming Field with Paddy Plants
Generates a complete farming environment with rice paddies, irrigation, and terrain
"""

import pybullet as p
import pybullet_data
import numpy as np
import os
import random
import math
import time

class FarmingFieldGenerator:
    def __init__(self, physics_client_id=0):
        """Initialize farming field generator"""
        self.physics_client = physics_client_id
        self.field_objects = []
        self.plant_bodies = []  # Track all plant bodies for removal
        
        # Field parameters
        self.field_size = 20.0
        self.paddy_field_size = 8.0
        self.plant_spacing = 1.5  # Increased spacing for better visibility
        self.row_spacing = 1.5
        
        # Plant growth stages (0-1, where 1 is fully grown)
        self.growth_stages = [0.3, 0.5, 0.7, 0.9, 1.0]
        
    def create_base_terrain(self):
        """Create the base terrain"""
        # Load plane as base
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        self.field_objects.append(("plane", plane_id))
        
        # Set proper physics properties for the ground
        p.changeDynamics(plane_id, -1, 
                        lateralFriction=0.8, 
                        rollingFriction=0.1, 
                        restitution=0.0,
                        contactStiffness=1e4,
                        contactDamping=1e3)
        
        # Create soil texture
        soil_color = [0.4, 0.25, 0.15, 1.0]
        p.changeVisualShape(plane_id, -1, rgbaColor=soil_color)
        
        print("[OK] Base terrain created with proper physics")
        return plane_id
    
    def create_paddy_field(self, position, size, water_level=0.05):
        """Create a paddy field with water"""
        # Create water surface
        water_height = water_level
        water_size = [size, size, water_height]
        
        # Create water box with proper physics
        water_collision_shape = p.createCollisionShape(
            p.GEOM_BOX, 
            halfExtents=[size/2, size/2, water_height/2]
        )
        
        water_visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[size/2, size/2, water_height/2],
            rgbaColor=[0.2, 0.4, 0.8, 0.7]  # Blue water
        )
        
        water_body = p.createMultiBody(
            baseMass=0,  # Static
            baseCollisionShapeIndex=water_collision_shape,
            baseVisualShapeIndex=water_visual_shape,
            basePosition=[position[0], position[1], water_height/2]
        )
        
        # Set water physics properties
        p.changeDynamics(water_body, -1,
                        lateralFriction=0.1,  # Low friction for water
                        rollingFriction=0.05,
                        restitution=0.0,
                        contactStiffness=1e3,
                        contactDamping=1e2)
        
        self.field_objects.append(("paddy_water", water_body))
        
        # Create field boundaries
        self.create_field_boundaries(position, size)
        
        print(f"[OK] Paddy field created at {position} with proper physics")
        return water_body
    
    def create_field_boundaries(self, position, size):
        """Create boundaries around the paddy field"""
        boundary_height = 0.15
        boundary_thickness = 0.1
        
        # Create four boundary walls
        boundary_positions = [
            [position[0] - size/2, position[1], boundary_height/2],  # Left
            [position[0] + size/2, position[1], boundary_height/2],  # Right
            [position[0], position[1] - size/2, boundary_height/2],  # Back
            [position[0], position[1] + size/2, boundary_height/2],  # Front
        ]
        
        boundary_sizes = [
            [boundary_thickness, size, boundary_height],  # Left/Right
            [size, boundary_thickness, boundary_height],  # Back/Front
        ]
        
        for i, pos in enumerate(boundary_positions):
            size_idx = 0 if i < 2 else 1
            
            boundary_collision = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[boundary_sizes[size_idx][0]/2, boundary_sizes[size_idx][1]/2, boundary_sizes[size_idx][2]/2]
            )
            
            boundary_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[boundary_sizes[size_idx][0]/2, boundary_sizes[size_idx][1]/2, boundary_sizes[size_idx][2]/2],
                rgbaColor=[0.3, 0.2, 0.1, 1.0]  # Brown
            )
            
            boundary_body = p.createMultiBody(
                baseMass=0,  # Static boundary
                baseCollisionShapeIndex=boundary_collision,
                baseVisualShapeIndex=boundary_visual,
                basePosition=pos
            )
            
            # Set boundary physics properties
            p.changeDynamics(boundary_body, -1,
                            lateralFriction=0.6,
                            rollingFriction=0.1,
                            restitution=0.0,
                            contactStiffness=1e4,
                            contactDamping=1e3)
            
            self.field_objects.append(("boundary", boundary_body))
    
    def create_simple_plant(self, position, growth_stage=1.0, plant_id=None):
        """Create a simple paddy plant using basic shapes with proper physics"""
        # Main stem
        stem_height = 0.6 * growth_stage
        stem_radius = 0.02  # Increased radius for better visibility
        
        stem_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=stem_radius,
            height=stem_height
        )
        
        stem_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=stem_radius,
            length=stem_height,
            rgbaColor=[0.2, 0.5, 0.2, 1.0]  # Green
        )
        
        # Calculate proper mass based on size
        stem_mass = 0.1 * growth_stage  # Increased mass for stability
        
        # Set plant name for tracking
        plant_name = f"paddy_plant_{plant_id}" if plant_id is not None else "paddy_plant"
        
        stem_body = p.createMultiBody(
            baseMass=stem_mass,
            baseCollisionShapeIndex=stem_collision,
            baseVisualShapeIndex=stem_visual,
            basePosition=[position[0], position[1], position[2] + stem_height/2]
        )
        
        # Set plant physics properties
        p.changeDynamics(stem_body, -1,
                        lateralFriction=0.3,
                        rollingFriction=0.1,
                        restitution=0.2,  # Some bounce
                        contactStiffness=1e3,
                        contactDamping=1e2,
                        linearDamping=0.1,  # Air resistance
                        angularDamping=0.1)
        
        # Add leaves if plant is growing
        if growth_stage > 0.3:
            self.add_plant_leaves(stem_body, growth_stage, plant_id)
        
        # Add rice panicle if mature
        if growth_stage > 0.8:
            self.add_rice_panicle(stem_body, growth_stage, plant_id)
        
        # Track this plant body
        self.plant_bodies.append({
            'body_id': stem_body,
            'name': plant_name,
            'position': position,
            'growth_stage': growth_stage,
            'plant_id': plant_id,
            'harvested': False
        })
        
        self.field_objects.append(("simple_plant", stem_body))
        return stem_body
    
    def add_plant_leaves(self, plant_body, growth_stage, plant_id=None):
        """Add leaves to the plant"""
        # Simplified leaf creation - just create visual leaves without constraints
        leaf_length = 0.3 * growth_stage
        leaf_width = 0.02
        
        # Create 4 leaves in different directions
        leaf_positions = [
            [0.15, 0, 0.4], [-0.15, 0, 0.4], [0, 0.15, 0.4], [0, -0.15, 0.4]
        ]
        
        for i, pos in enumerate(leaf_positions):
            leaf_visual = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[leaf_length/2, leaf_width/2, 0.0025],
                rgbaColor=[0.1, 0.6, 0.1, 1.0]  # Dark green
            )
            
            # Create leaf as separate body with light mass
            leaf_name = f"leaf_{plant_id}_{i}" if plant_id is not None else f"leaf_{i}"
            leaf_body = p.createMultiBody(
                baseMass=0.005,  # Very light
                baseVisualShapeIndex=leaf_visual,
                basePosition=pos
            )
            
            # Set leaf physics properties
            p.changeDynamics(leaf_body, -1,
                            lateralFriction=0.2,
                            rollingFriction=0.05,
                            restitution=0.1,
                            linearDamping=0.2,  # More air resistance for leaves
                            angularDamping=0.2)
            
            self.field_objects.append(("leaf", leaf_body))
    
    def add_rice_panicle(self, plant_body, growth_stage, plant_id=None):
        """Add rice panicle (grain cluster) to mature plants"""
        panicle_radius = 0.08 * growth_stage
        
        panicle_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=panicle_radius,
            rgbaColor=[0.8, 0.7, 0.2, 1.0]  # Golden
        )
        
        panicle_name = f"panicle_{plant_id}" if plant_id is not None else "panicle"
        panicle_body = p.createMultiBody(
            baseMass=0.01,  # Light mass for panicle
            baseVisualShapeIndex=panicle_visual,
            basePosition=[0, 0, 0.65 * growth_stage]
        )
        
        # Set panicle physics properties
        p.changeDynamics(panicle_body, -1,
                        lateralFriction=0.4,
                        rollingFriction=0.1,
                        restitution=0.3,  # More bounce for grains
                        linearDamping=0.15,
                        angularDamping=0.15)
        
        self.field_objects.append(("panicle", panicle_body))
    
    def create_plant_grid(self, field_position, field_size, plant_spacing=1.5):
        """Create a grid of paddy plants"""
        plants = []
        
        # Create a 3x3 grid for the harvester demo
        plants_per_row = 3
        num_rows = 3
        
        plant_counter = 0
        
        # Create plants in a grid pattern
        for row in range(num_rows):
            for col in range(plants_per_row):
                # Calculate plant position
                x = field_position[0] - 1.5 + col * plant_spacing
                y = field_position[1] - 1.5 + row * plant_spacing
                z = 0.05  # Slightly above water level
                
                # Random growth stage for realistic variation
                growth_stage = random.choice(self.growth_stages)
                
                # Add some randomness to positions
                x += random.uniform(-0.1, 0.1)
                y += random.uniform(-0.1, 0.1)
                
                plant_id = self.create_simple_plant([x, y, z], growth_stage, plant_counter)
                plants.append(plant_id)
                plant_counter += 1
        
        print(f"[OK] Created {len(plants)} paddy plants in grid")
        return plants
    
    def create_irrigation_channel(self, position, length, width=0.5, depth=0.3):
        """Create an irrigation channel"""
        channel_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[width/2, length/2, depth/2]
        )
        
        channel_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[width/2, length/2, depth/2],
            rgbaColor=[0.1, 0.3, 0.6, 0.8]  # Dark blue water
        )
        
        channel_body = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=channel_collision,
            baseVisualShapeIndex=channel_visual,
            basePosition=[position[0], position[1], depth/2]
        )
        
        self.field_objects.append(("irrigation_channel", channel_body))
        print(f"[OK] Irrigation channel created at {position}")
        return channel_body
    
    def create_terrain_features(self):
        """Create additional terrain features"""
        # Create small hills
        hill_positions = [
            [0, 8, 0],
            [-8, -8, 0],
            [8, -8, 0]
        ]
        
        for pos in hill_positions:
            hill_radius = random.uniform(0.8, 1.5)
            hill_height = random.uniform(0.3, 0.8)
            
            hill_collision = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=hill_radius
            )
            
            hill_visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=hill_radius,
                rgbaColor=[0.2, 0.5, 0.2, 1.0]  # Grass green
            )
            
            hill_body = p.createMultiBody(
                baseMass=50.0,
                baseCollisionShapeIndex=hill_collision,
                baseVisualShapeIndex=hill_visual,
                basePosition=[pos[0], pos[1], hill_height]
            )
            
            self.field_objects.append(("hill", hill_body))
        
        print("[OK] Terrain features created")
    
    def get_plant_bodies(self):
        """Get all plant bodies for tracking"""
        return self.plant_bodies
    
    def remove_plant(self, plant_body_id):
        """Remove a plant from the environment"""
        try:
            # Find the plant in our tracking list
            for plant in self.plant_bodies:
                if plant['body_id'] == plant_body_id and not plant['harvested']:
                    # Remove the plant body from PyBullet
                    p.removeBody(plant_body_id)
                    plant['harvested'] = True
                    print(f"✓ Removed plant {plant['name']} from environment")
                    return True
        except Exception as e:
            print(f"Error removing plant: {e}")
        return False
    
    def create_complete_farming_field(self):
        """Create a complete realistic farming field"""
        print("Creating realistic farming field...")
        
        # Create base terrain
        self.create_base_terrain()
        
        # Create two paddy fields
        paddy_field_1 = self.create_paddy_field([-5, 0, 0], self.paddy_field_size)
        paddy_field_2 = self.create_paddy_field([5, 0, 0], self.paddy_field_size)
        
        # Create irrigation channel between fields
        self.create_irrigation_channel([0, 0, 0], 20.0)
        
        # Create plant grids in both fields
        plants_1 = self.create_plant_grid([-5, 0, 0], self.paddy_field_size)
        plants_2 = self.create_plant_grid([5, 0, 0], self.paddy_field_size)
        
        # Create terrain features
        self.create_terrain_features()
        
        print(f"[SUCCESS] Farming field created with {len(plants_1) + len(plants_2)} paddy plants")
        print(f"[INFO] Plant tracking enabled - {len(self.plant_bodies)} plants tracked")
        
        return {
            'paddy_fields': [paddy_field_1, paddy_field_2],
            'plants': plants_1 + plants_2,
            'plant_bodies': self.plant_bodies,
            'objects': self.field_objects
        }

def main():
    """Main function to create farming field"""
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setGravity(0, 0, -9.81)
    
    # Set camera position for better view
    p.resetDebugVisualizerCamera(
        cameraDistance=15,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Create farming field
    generator = FarmingFieldGenerator(physics_client)
    field_data = generator.create_complete_farming_field()
    
    print("\nFarming Field Features:")
    print("- 2 Paddy fields with water")
    print("- Irrigation channel")
    print(f"- {len(field_data['plants'])} Paddy plants with varying growth stages")
    print("- Terrain features (hills)")
    print("- Realistic soil and water textures")
    print("- Plant tracking enabled for harvesting")
    
    # Keep simulation running
    try:
        while True:
            p.stepSimulation()
            time.sleep(1.0/240.0)
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    except:
        print("\nSimulation ended")
    
    p.disconnect()

if __name__ == "__main__":
    main()
