import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';
import { FaRobot, FaBrain } from 'react-icons/fa';
import { IoChatbubbles } from 'react-icons/io5';

const FeatureList = [
  {
    title: 'ROS 2 & Robotics Foundations',
    Icon: FaRobot,
    description: (
      <>
        Dive into the core concepts of ROS 2, covering nodes, topics, services,
        actions, and building robotic simulations with URDF and Gazebo.
      </>
    ),
  },
  {
    title: 'Advanced Simulation & AI Integration',
    Icon: FaBrain,
    description: (
      <>
        Explore advanced simulation environments like Unity and master NVIDIA Isaac
        for AI-driven robotics, including Isaac Sim and Isaac ROS.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action Models',
    Icon: IoChatbubbles,
    description: (
      <>
        Learn about cutting-edge VLA models, multi-modal perception, conversational
        robotics, and how to implement LLM-driven task planning for humanoid robots.
      </>
    ),
  },
];

function Feature({Icon, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className="text--center">
          <Icon className={styles.featureSvg} />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3">{title}</Heading>
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}