<!-- omit in toc -->

# Jira

Jira is a platform for software development by Atlassian, used by big companies (Bosch, etc.).

It has templates for Scrum, Kanban, etc.

<!-- omit in toc -->

## Table of Contents

- [Overview](#overview)
- [Agile Project Management](#agile-project-management)
  - [Common Terms](#common-terms)
  - [Stakeholders](#stakeholders)
- [Project Types](#project-types)
- [Jira Software Management](#jira-software-management)
  - [Terms](#terms)
  - [Story Points](#story-points)
- [Confluence](#confluence)
- [Tips and Tricks](#tips-and-tricks)

---

## Overview

- [Jira in a Nutshell demo video](https://youtu.be/xrCJv0fTyR8)
- [What is Atlassian?](https://youtu.be/hWXNmcSN4bE)
- [Confluence Demonstration](https://youtu.be/uhWCMlcY_Zw)
- [Introduction to JIRA & Agile Project Management](https://youtu.be/NrHpXvDXVrw)

---

## Agile Project Management

- Agile Project Management, or Agile Software Development, is just an umbrella term for a **_"set of frameworks and practices"_**
- These frameworks and practices:

  - (usually) support the development and delivery of software package
  - break down complex projects into small manageable goals
  - deliver the product to the customers in increments

  E.g.: a customer want a new website for his restaurant, which can be broken down into:

  - Decide on website functionality
  - Collect data (menu, contact, etc.)
  - Build the website
  - The product (website) is delivered in increments, continuously adding functionalities based on customer's needs.
    1. Build the draft website
    2. Add menu
    3. Add contact details
    4. Add customer reviews, ranking

- For short, just call it "Agile".
- There are many frameworks, the common ones are:
  - Scrum
  - Kanban
  - etc.

### Common Terms

- Sprint (for Scrum) is a time box (2-4 weeks), during which, the team aims to finish a specified amount of work.
- Product Backlog is the list of features that the product owner have from taking the input from customers and other stakeholders.
- User story: As a [user], I want [feature] so that I can [benefit].
  - E.g.:
    - As a Jira admin, I want to be able to add member so that I can assign them to tasks.
    - As a iPhone user, I want to have voice control over application so that I spend less effort.
  - No technical stuff is mentioned in user story, because it doesn't matter to the customer how the technology works.\
    It's the developers' job to figure it out how to do it.
  - Epic is a story that is too big for a single sprint. Epic will be broken down into multiple stories
  - Story is a story that can be complete during a single sprint
- WIP (work in progress) is the number of tasks that are in the IN PROGRESS column.

### Stakeholders

- Product owner decides what to build, takes feedback from customers and other stakeholders into the Product Backlog.
- Development team builds it.
- Customers use it and benefit from it.
- Scrum master / Agile coach is the person who helps the product owner and the dev team to adopt and maintain good habit.

---

## Project Types

|            | Scrum                                       | Kanban                                  |
| ---------- | ------------------------------------------- | --------------------------------------- |
| Modulation | Regular fixed length sprints                | Continuous flow (adding tasks)          |
| Release    | At the end of sprint                        | Continuous delivery                     |
| Roles      | Mandatory roles (owner, scrum master, etc.) | An agile coach (but not necessary)      |
| Planning   | At the beginning of sprint                  | Just-in-time planning                   |
| WIP Limits | The number of tasks during a sprint         | The number of IN PROGRESS tasks         |
| Changes    | During a sprint, no change is allowed       | Can constantly add tasks to the backlog |
| KPI(s)     | Velocity (number of story points)           | Lead time, cycle time                   |
| For what   | Move fast but need some coordination        | Have a lot of incoming tasks            |
|            | Goal-driven projects                        | Tasks with changing priorities          |
| For whom   | Non-mature teams                            | Mature teams                            |

---

## Jira Software Management

### Terms

Source: [Simon Sez IT](https://youtu.be/nHuhojfjeUY)

- Issues are containers for fields (name, description, summary, assignee, due date)\
  Different issue types: Epic, Story, Bug, etc.
- Projects are containers for issues.

### Story Points

The amount of story points depends on:

- The effort/time/volume
- Task complexity
- Uncertainty: the given story from the customer is unclear
- Risk: implementing new feature requires to change some old, not well documented or well tested code

---

## Confluence

Confluence is the documentation tool for collaboration by Atlassian.

---

## Tips and Tricks

Tips from [Dan Chuparkoff](https://youtu.be/NrHpXvDXVrw):

- Define a task:
  - More than 30 minutes
  - Less than 3 days (even less than 1 day)
  - Waiting during a task is a sign that the task is actually a collection of tasks
- Estimating story points

  | 30mins  |  1 story point  |
  | :-----: | :-------------: |
  | 1 hour  | 2 story points  |
  | 2 hours | 4 story points  |
  | 4 hours | 8 story points  |
  |  1 day  | 16 story points |
  | 2 days  | 32 story points |
  | 3 days  | 48 story points |

- TODO
