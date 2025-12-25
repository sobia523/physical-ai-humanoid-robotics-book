# Quickstart Guide: Professional Docusaurus UI System

## Overview

This quickstart guide provides the essential steps to set up, customize, and run the Professional Docusaurus UI System for the Physical AI & Humanoid Robotics textbook. Follow these steps to get the themed documentation site running with the new UI/UX.

## Prerequisites

Before starting, ensure you have:

- Node.js version 18 or higher
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Step 1: Environment Setup

### Clone the Repository
```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotics-Textbook
```

### Install Dependencies
```bash
npm install
# or
yarn install
```

### Verify Docusaurus Installation
```bash
npx docusaurus --version
```

## Step 2: Project Structure Understanding

Familiarize yourself with the key directories:

```
Physical-AI-Humanoid-Robotics-Textbook/
├── docs/                    # Documentation content (modules, chapters)
├── src/                     # Custom source files (components, pages, CSS)
├── static/                  # Static assets (images, files)
├── docusaurus.config.js     # Docusaurus configuration
├── sidebars.js              # Navigation configuration
└── package.json             # Project dependencies and scripts
```

## Step 3: Theme Configuration

### Update Docusaurus Configuration

In `docusaurus.config.js`, ensure the following theme settings:

```javascript
module.exports = {
  // ... other config
  themes: [
    '@docusaurus/theme-classic',
    '@docusaurus/theme-search-algolia', // for search functionality
  ],
  themeConfig: {
    navbar: {
      // Custom navbar will be implemented via swizzling
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
    },
  },
};
```

### Add Custom CSS Variables

Create or update `src/css/custom.css` with the theme color variables:

```css
:root {
  /* Light mode theme */
  --ifm-color-primary: #2563EB;
  --ifm-color-primary-dark: #1D4ED8;
  --ifm-background-color: #FFFFFF;
  --ifm-background-surface-color: #F8FAFC;
  --ifm-text-color: #0F172A;
  --ifm-text-color-secondary: #475569;
  --ifm-border-color: #E2E8F0;
  --ifm-card-hover-background: #F1F5F9;
}

[data-theme='dark'] {
  /* Dark mode theme */
  --ifm-color-primary: #3B82F6;
  --ifm-color-primary-dark: #60A5FA;
  --ifm-background-color: #0B1120;
  --ifm-background-surface-color: #111827;
  --ifm-text-color: #E5E7EB;
  --ifm-text-color-secondary: #9CA3AF;
  --ifm-border-color: #1F2937;
  --ifm-card-hover-background: #1E293B;
}
```

## Step 4: Component Customization

### Swizzle Required Components

To customize the homepage, navbar, and other components, use Docusaurus swizzling:

```bash
# Swizzle the homepage component
npx docusaurus swizzle @docusaurus/theme-classic DocPage -- --eject

# Swizzle the navbar component
npx docusaurus swizzle @docusaurus/theme-classic Navbar -- --eject

# Swizzle the footer component
npx docusaurus swizzle @docusaurus/theme-classic Footer -- --eject
```

### Create Custom Homepage

Create `src/pages/index.js` with the hero section:

```javascript
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">Physical AI & Humanoid Robotics</h1>
        <p className="hero__subtitle">Embodied Intelligence for the Real World</p>
        <p>Explore the integration of physical intelligence, humanoid systems, ROS 2, simulation, perception, planning, and real-world deployment in advanced robotics.</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/modules/001-introduction">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Professional Docusaurus UI for Physical AI & Humanoid Robotics textbook">
      <HomepageHeader />
      <main>
        {/* Module cards will be added here */}
      </main>
    </Layout>
  );
}
```

### Create Module Cards Component

Create `src/components/ModuleCards/index.js`:

```javascript
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

const ModuleList = [
  {
    title: 'Module 1: Introduction',
    description: 'Fundamentals of Physical AI and Humanoid Robotics',
    to: '/docs/modules/001-introduction',
  },
  {
    title: 'Module 2: Perception',
    description: 'Sensing and understanding the physical world',
    to: '/docs/modules/002-perception',
  },
  {
    title: 'Module 3: Planning',
    description: 'Motion and task planning for humanoid systems',
    to: '/docs/modules/003-planning',
  },
  {
    title: 'Module 4: VLA Systems',
    description: 'Vision-Language-Action systems integration',
    to: '/docs/modules/004-vla-systems',
  },
];

function ModuleCard({title, description, to}) {
  return (
    <div className={clsx('col col--6')}>
      <div className="card demo-card">
        <div className="card__body">
          <h3>{title}</h3>
          <p>{description}</p>
        </div>
        <div className="card__footer">
          <Link to={to} className="button button--primary">
            Explore Module
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function ModuleCards() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <div className="row">
          {ModuleList.map((props, idx) => (
            <ModuleCard key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
```

Add corresponding CSS in `src/components/ModuleCards/styles.module.css`:

```css
.modules {
  padding: 4rem 0;
  background-color: var(--ifm-background-surface-color);
}

@media screen and (max-width: 996px) {
  .modules {
    padding: 2rem 0;
  }
}

.demo-card {
  height: 100%;
  transition: all var(--ifm-transition-fast) ease;
  transition-property: border, box-shadow;
  border: 1px solid var(--ifm-border-color);
}

.demo-card:hover {
  border-color: var(--ifm-color-primary);
  background-color: var(--ifm-card-hover-background);
  cursor: pointer;
}

.demo-card .card__footer {
  text-align: center;
}
```

## Step 5: Enhanced Search Implementation

### Configure Search Plugin

Update your `docusaurus.config.js` to enhance search functionality:

```javascript
module.exports = {
  // ... other config
  themes: [
    [
      '@docusaurus/theme-search-algolia',
      {
        // ... other search config
        contextualSearch: true,
        searchParameters: {},
        searchPagePath: 'search',
      },
    ],
  ],
};
```

### Custom Search Component (Optional)

If you need to customize search behavior further, create a custom search component that adheres to the search contract specifications.

## Step 6: Responsive Design Implementation

### Module Grid CSS

Add responsive grid styling to ensure proper module card layout:

```css
/* In src/css/custom.css or component-specific CSS */

.module-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1.5rem;
  width: 100%;
}

@media (max-width: 768px) {
  .module-grid {
    grid-template-columns: 1fr;
  }
}

/* For tablet (769px - 1024px), maintain 2 columns */
@media (min-width: 769px) and (max-width: 1024px) {
  .module-grid {
    grid-template-columns: repeat(2, 1fr);
  }
}
```

## Step 7: Run the Development Server

Start the development server with hot reloading:

```bash
npm run start
# or
yarn start
```

The site will be available at `http://localhost:3000` (or alternative port if 3000 is busy).

## Step 8: Build for Production

To create a production build:

```bash
npm run build
# or
yarn build
```

The built site will be in the `build/` directory and can be served statically.

## Step 9: Testing and Validation

### Performance Testing

Verify that:
- Page load times are under 3 seconds
- Search responds in under 1 second
- Theme switching is instantaneous

### Accessibility Testing

Ensure:
- WCAG 2.1 AA compliance
- Proper color contrast ratios
- Keyboard navigation support
- Screen reader compatibility

### Responsive Testing

Test on:
- Mobile devices (320px - 768px)
- Tablets (769px - 1024px)
- Desktops (1025px+)

## Troubleshooting

### Common Issues

1. **Port Already in Use**: If `npm run start` fails due to port conflicts, use:
   ```bash
   npm run start -- --port 8083
   ```

2. **MDX Compilation Errors**: Ensure no angle brackets `<` are used directly in markdown files. Use `&lt;` instead.

3. **Theme Not Applying**: Verify CSS variables are properly defined and theme configuration is correct.

4. **Search Not Working**: Check Algolia configuration or local search plugin setup.

### Verification Steps

After setup, verify:
- [ ] Homepage displays hero section correctly
- [ ] Module cards show with proper descriptions
- [ ] Theme switching works between light/dark modes
- [ ] Navigation links work correctly
- [ ] Search functions across modules and chapters
- [ ] Responsive layout works on all screen sizes
- [ ] All existing content remains accessible

## Next Steps

1. Customize additional components as needed
2. Add any specific styling refinements
3. Implement analytics if required
4. Set up deployment pipeline
5. Document any additional customizations