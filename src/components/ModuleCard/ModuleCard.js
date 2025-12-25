import React from 'react';
import Link from '@docusaurus/Link';
import clsx from 'clsx';

const ModuleCard = ({ module }) => {
  // Extract module number from the title (e.g., "Module 1: Introduction" -> "Module 1")
  const moduleNumber = module.title.split(':')[0] || module.title;
  const moduleTitle = module.title.includes(':') ? module.title.split(':').slice(1).join(':').trim() : module.title;

  return (
    <Link
      to={module.path}
      className={clsx('module-card')}
      style={{ textDecoration: 'none' }}
    >
      <div className="module-card__number">{moduleNumber}</div>
      <h3 className="module-card__title">{moduleTitle}</h3>
      <p className="module-card__description">{module.description}</p>
      <button className={clsx('button button--primary module-card__button')}>
        Open Module
      </button>
    </Link>
  );
};

export default ModuleCard;