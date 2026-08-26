#!/usr/bin/env node
/**
 * Embeds welcome-*.md files into <profile>/config.yaml for the welcome dashboard text widgets.
 * Run after editing the .md files, once per profile (the Action widget differs per profile:
 * welcome-action.md for ros2, welcome-action-iso.md for iso):
 *
 *   node embed-welcome-content.mjs ros2
 *   node embed-welcome-content.mjs iso
 */

import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const profile = process.argv[2];
if (!['ros2', 'iso'].includes(profile)) {
  console.error('usage: node embed-welcome-content.mjs ros2|iso');
  process.exit(1);
}
const configPath = path.join(__dirname, profile, 'config.yaml');

const sections = [
  {
    label: '',
    widgets: [
      { file: 'welcome-intro.md', label: 'Welcome', grid: 12 },
    ],
  },
  {
    label: 'Dashboards',
    widgets: [
      { file: 'welcome-fleet.md', label: 'Fleet', grid: 4 },
      { file: 'welcome-robot.md', label: 'Robot', grid: 4 },
      { file: 'welcome-navigation.md', label: 'Navigation', grid: 4 },
    ],
  },
  {
    label: 'Simulation',
    widgets: [
      { file: 'welcome-simulation-map.md', label: 'Map', grid: 6 },
      { file: profile === 'iso' ? 'welcome-action-iso.md' : 'welcome-action.md', label: 'Action', grid: 6 },
      { file: 'welcome-footer.md', label: 'Footer', grid: 12 },
    ],
  },
];

const indent = (text, spaces) =>
  text.split('\n').map(line => ' '.repeat(spaces) + line).join('\n');

const renderWidget = ({ file, label, grid }) => {
  const text = fs.readFileSync(path.join(__dirname, file), 'utf8').replace(/\n$/, '');
  return `    - type: text
      label: ${label}
      config:
        text: |
${indent(text, 10)}
      layout:
        chroma: false
        grid: ${grid}`;
};

const sectionsBlock = `  sections:
${sections.map(({ label, widgets }) => `  - label: '${label}'
    scope: robot
    withControlWidget: false
    widgets:
${widgets.map(renderWidget).join('\n')}`).join('\n')}`;

let config = fs.readFileSync(configPath, 'utf8');
const welcomeStart = config.indexOf('metadata:\n  id: welcome');
if (welcomeStart === -1) {
  console.error('Could not find welcome dashboard in config.yaml');
  process.exit(1);
}

const sectionsStart = config.indexOf('  sections:', welcomeStart);
if (sectionsStart === -1) {
  console.error('Could not find sections block in welcome dashboard');
  process.exit(1);
}

const nextDoc = config.indexOf('\n---', sectionsStart);
const sectionsEnd = nextDoc === -1 ? config.length : nextDoc;

config =
  config.slice(0, sectionsStart) +
  sectionsBlock +
  '\n' +
  config.slice(sectionsEnd);

fs.writeFileSync(configPath, config);
console.log(`Updated welcome text widgets in ${profile}/config.yaml from .md files`);
