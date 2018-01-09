# -*- coding: utf-8 -*-
# Generated by Django 1.11.2 on 2017-06-14 11:22
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    initial = True

    dependencies = [
        ('account', '0001_initial'),
    ]

    operations = [
        migrations.CreateModel(
            name='Dashboard',
            fields=[
                ('name', models.CharField(max_length=128, primary_key=True, serialize=False)),
                ('state', models.TextField()),
                ('owners', models.ManyToManyField(related_name='dashboards', to='account.Profile')),
            ],
        ),
        migrations.CreateModel(
            name='Template',
            fields=[
                ('name', models.CharField(max_length=128, primary_key=True, serialize=False)),
                ('state', models.TextField()),
                ('owners', models.ManyToManyField(related_name='templates', to='account.Profile')),
            ],
        ),
    ]
